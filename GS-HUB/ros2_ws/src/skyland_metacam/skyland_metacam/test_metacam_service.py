#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

# 服务与消息类型
from hub_interface.srv import DistortionRemoval, SemanticInfo
from hub_interface.msg import SensoryData
from .utils.pc_utils import extract_point_cloud_to_nparray
from common_utils import serialize_response, unserialize_response

from sensor_msgs.msg import CompressedImage, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import cv2
import os
import numpy as np


class SensorySubscriberClientNode(Node):
    def __init__(self):
        super().__init__('sensory_subscriber_client_node')

        # 创建服务客户端：畸变矫正
        self.remove_distortion_client = self.create_client(DistortionRemoval, '/gs_hub/remove_distortion')
        self.get_logger().info('Waiting for "remove_distortion" service...')
        self.remove_distortion_client.wait_for_service()
        self.get_logger().info('"remove_distortion" service is available!')

        # 创建服务客户端：生成语义信息
        self.generate_semantic_info_client = self.create_client(SemanticInfo, '/gs_hub/gen_semantic_info')
        self.get_logger().info('Waiting for "gen_semantic_info" service...')
        self.generate_semantic_info_client.wait_for_service()
        self.get_logger().info('"gen_semantic_info" service is available!')

        # 订阅 /gs_hub/sensory_data 话题
        self.subscription = self.create_subscription(
            SensoryData,
            '/gs_hub/sensory_data',
            self.sensory_data_callback,
            10
        )


    def sensory_data_callback(self, msg: SensoryData):
        """
        当接收到 /gs_hub/sensory_data 消息时触发此回调。
        msg 中包含:
          - left_image
          - right_image
          - point_cloud
          - odometry
          - l2w_matrix
        """

        # 1) 选择要处理的图像（可以灵活选择 left_image 或 right_image）
        #    假设此处我们要使用 left_image
        camera_name = "left"
        image_to_process = msg.left_image

        # 2) 调用 remove_distortion 服务对图像进行畸变矫正
        #    传入 camera_name (left/right) + 对应的 compressed image
        distorted_image = image_to_process

        # 调用 remove_distortion
        self.call_remove_distortion_service(camera_name, distorted_image).add_done_callback(
            lambda future: self.handle_remove_distortion_response(
                future, 
                msg.point_cloud,
                msg.odometry,
                camera_name
            )
        )

    def call_remove_distortion_service(self, camera_name_str: str, compressed_img: CompressedImage):
        """异步调用 remove_distortion 服务"""
        request = DistortionRemoval.Request()
        request.camera_name.data = camera_name_str
        request.image = compressed_img
        return self.remove_distortion_client.call_async(request)

    def handle_remove_distortion_response(self, future, point_cloud: PointCloud2, odometry: Odometry, camera_name_str: str):
        """
        处理 remove_distortion 服务返回的响应，并进一步调用 semantic_info 服务
        """
        try:
            response = future.result()
            self.get_logger().info("remove distortion success")
        except Exception as e:
            self.get_logger().error(f"RemoveDistortion Service call failed: {e}")
            return

        # 获取去畸变后的图像
        processed_image = response.processed_image

        # save debug
        # np_arr = np.frombuffer(processed_image.data, np.uint8)
        # saved_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # try:
        #     output_dir = "processed_image_sbsbs"
        #     os.makedirs(output_dir, exist_ok=True)
        #     output_path = os.path.join(output_dir, f"{camera_name_str}_processed_image.jpg")
            
        #     cv2.imwrite(output_path, saved_image)
        #     self.get_logger().info(f"Processed image saved at: {output_path}")
        # except Exception as save_error:
        #     self.get_logger().error(f"Failed to save processed image: {save_error}")

        self.call_semantic_info_service(
            processed_image,
            text="What objects are here?",
            point_cloud=point_cloud,
            odometry=odometry,
            camera_name_str=camera_name_str
        ).add_done_callback(self.handle_semantic_info_response)

    def call_semantic_info_service(
        self,
        processed_image: CompressedImage,
        text: str,
        point_cloud: PointCloud2,
        odometry: Odometry,
        camera_name_str: str
    ):
        """
        异步调用 gen_semantic_info 服务
        """
        request = SemanticInfo.Request()
        request.image = processed_image
        request.text.data = text
        request.point_cloud = point_cloud
        request.odometry = odometry
        request.camera_name.data = camera_name_str
        return self.generate_semantic_info_client.call_async(request)

    def handle_semantic_info_response(self, future):
        """
        处理 gen_semantic_info 服务返回的响应
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"SemanticInfo Service call failed: {e}")
            return

        # 打印结果
        self.get_logger().info(f"Semantic Info -> success: {response.success}, msg: {response.msg.data}")
        if response.success:
            data = unserialize_response(response.data.data)
            del data['image_entity']
            self.get_logger().info(f"Data: {data}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorySubscriberClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()