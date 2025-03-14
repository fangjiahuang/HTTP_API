
import rclpy
import os
import shutil
import time
import uuid

from rclpy.node import Node
from qdrant_client import QdrantClient
from common_utils import ConfigManager
from queue import Queue
from sensor_msgs.msg import CompressedImage, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from typing import List

from . import CONFIG_PATH
from .semantic_controller.semantic_manager import SemanticManger
from hub_interface.msg import SensoryData, SemanticData
from hub_interface.srv import DistortionRemoval, SemanticInfo
from hub_interface.srv import StartMapBuilder, EndMapBuilder, StartGenSemantic, EndGenSemantic
from common_utils import serialize_response, unserialize_response
from .service.text_encoder import text_encoder
from .utils.service_utils import prepare_distortion_request, prepare_semantic_request


class SemanticMapManager(Node):

    def __init__(self, configs):
        super().__init__('semantic_map_manager')
        self.t2v_endpoint = configs['models'].get('text_encoder_url')
     
        self.build_in_progress = False

        self.__init_semantic_server()
        self.__init_topic_suscriber()
        self.__init_client()
        self.__init_publisher()

    def __init_semantic_server(self):
        self.build_db_service = self.create_service(
            srv_type=StartGenSemantic,
            srv_name='/gs_hub/start_gen_semantic',
            callback=self.start_gen_semantic_info_callback
        )

        self.end_build_db_service = self.create_service(
            srv_type=EndGenSemantic,
            srv_name='/gs_hub/end_gen_semantic',
            callback=self.end_gen_semantic_info_callback
        )

    def __init_client(self):
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

    def __init_topic_suscriber(self):
        # 订阅 /gs_hub/sensory_data 话题
        self.subscription = self.create_subscription(
            SensoryData,
            '/gs_hub/sensory_data',
            self.sensory_data_callback,
            10
        )
        self.get_logger().info("Subscribe: '/gs_hub/sensory_data'")
    
    def __init_publisher(self):
        self.left_semantic_data_pub = self.create_publisher(SemanticData, '/gs_hub/semantic_data/left', 10)
        self.right_semantic_data_pub = self.create_publisher(SemanticData, '/gs_hub/semantic_data/right', 10)

    def publish_semantic_data(
        self,
        camera_name: str,
        image_b64: str,
        caption: str,
        position: List,
        vector: List,
        confidence: float
    ):
        semantic_msg = SemanticData()
        semantic_msg.id.data = str(uuid.uuid4()) 
        semantic_msg.image.data = image_b64
        semantic_msg.camera_name.data = camera_name
        semantic_msg.caption.data = caption

        semantic_msg.position = position
        semantic_msg.vector = vector
        
        semantic_msg.confidence = confidence

        if camera_name == "left":
             self.left_semantic_data_pub.publish(semantic_msg)
        elif camera_name == "right":
            self.right_semantic_data_pub.publish(semantic_msg)

    def sensory_data_callback(self, msg):
        """
        每当收到 /gs_hub/sensory_data 数据时，如果 build_in_progress = True，
        """
        if not self.build_in_progress:
            return 
        
        self.convert_raw2sementic_msg(msg)

    def convert_raw2sementic_msg(self,
        msg: SensoryData
    ):  

        if msg.left_image.data:
            self.get_logger().info("Calling remove_distortion for LEFT image ...")
            self.call_remove_distortion_service("left", msg.left_image).add_done_callback(
                lambda future: self.handle_remove_distortion_response(
                    future, 
                    msg.point_cloud,
                    msg.odometry,
                    "left"
                )
            )

        if msg.right_image.data:
            self.get_logger().info("Calling remove_distortion for RIGHT image ...")
            self.call_remove_distortion_service("right", msg.right_image).add_done_callback(
                lambda future: self.handle_remove_distortion_response(
                    future, 
                    msg.point_cloud,
                    msg.odometry,
                    "right"
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
            self.get_logger().info(f"remove distortion success for {camera_name_str}")
        except Exception as e:
            self.get_logger().error(f"RemoveDistortion Service call failed for {camera_name_str}: {e}")
            return

        # 获取去畸变后的图像
        processed_image = response.processed_image
       
        self.call_semantic_info_service(
            processed_image,
            text="",
            point_cloud=point_cloud,
            odometry=odometry,
            camera_name_str=camera_name_str
        ).add_done_callback(
            lambda future:self.handle_semantic_info_response(
                        future, 
                        camera_name_str
            ) 
        )
    
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

    def handle_semantic_info_response(self, future, camera_name: str):
        """
        处理 gen_semantic_info 服务返回的响应
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"SemanticInfo Service call failed: {e}")
            return

        # 打印结果
        self.get_logger().info(f"Semantic Info -> {camera_name} success: {response.success}")
        if response.success:
            data = unserialize_response(response.data.data)
            data = data["data"] # List of semantic info
            if len(data) == 0:
                return future.result()
            caption_list = [info_res['caption'] for info_res in data]
            print("caption list", caption_list)
            embeddings = text_encoder(self.t2v_endpoint, caption_list)
            for idx, info_res in enumerate(data): 
                self.publish_semantic_data(
                    camera_name=camera_name,
                    image_b64=info_res['image_entity'],
                    caption=info_res['caption'],
                    position=info_res['position'],
                    vector=embeddings[idx],
                    confidence=info_res['confidence'],
                )
        
            return future.result()
            
        return
 
    def start_gen_semantic_info_callback(self, request, response):
    
        self.build_in_progress = True
        
        self.get_logger().info(f"Generation started.")

        response.success = True
        response.msg.data = f"Start generating semantic info."
        return response
    
    def end_gen_semantic_info_callback(self, request, response):

        self.build_in_progress = False

        self.get_logger().info("Generation ended. No longer receiving data.")
        
        response.success = True
        response.msg.data = "Generation ended successfully."
        return response

def main():
    rclpy.init()
    node = None
    try:
        config_manager = ConfigManager()
        semantic_config = config_manager.get_config(filename = CONFIG_PATH)

        node = SemanticMapManager(semantic_config)
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')

    except Exception as e:
        if node is not None:
            node.get_logger().error(f"An unexpected error occurred: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
