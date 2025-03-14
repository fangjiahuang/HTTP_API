from common_utils import ConfigManager, serialize_response
from hub_interface.srv import DistortionRemoval, SemanticInfo
from .utils.image_utils import load_camera_data, remove_fisheyes, encode_image_bytes_to_base64
from .utils.points_fusion import PointsFusion
from .utils.odometry_utils import odometry_to_transform
from .utils.pc_utils import extract_point_cloud_to_nparray
from .service.object_detection_service import object_detection_predict
from . import CONFIG_PATH

import rclpy
import cv2
import json
import random
import numpy as np

from rclpy.node import Node
from io import BytesIO
from sensor_msgs.msg import CompressedImage

class MetacamService(Node):
    def __init__(self, configs):
        super().__init__('metacam_service')
        self.camera_data, self.metacam_obj = load_camera_data(configs['skyland_metacam'].get('calibration_path'))
        
        self.pf = PointsFusion(self.metacam_obj)
 
        self.remove_distortion_srv = self.create_service(DistortionRemoval, '/gs_hub/remove_distortion', self.remove_distortion_callback)
        self.get_logger().info("add remove_distortion_srv")
        self.semantic_info_srv = self.create_service(SemanticInfo, "/gs_hub/gen_semantic_info", self.generate_semantic_info_callback)
        self.get_logger().info("add gen_semantic_info")

    def remove_distortion_callback(self, request, response):
        self.get_logger().info("Start distortion remove")
        try:
            camera_name = request.camera_name.data
            fisheye_image = request.image.data
            dealt_camera_data = next(cam for cam in self.camera_data['cameras'] if cam['name'] == camera_name)
            decoded_fisheye_image = cv2.imdecode(np.frombuffer(fisheye_image, np.uint8), cv2.IMREAD_COLOR)
            
            undistorted_image = remove_fisheyes(dealt_camera_data, decoded_fisheye_image)
            
            ret, encoded_image = cv2.imencode('.jpg', undistorted_image)
            if not ret:
                raise RuntimeError("Failed to encode the undistorted image.")

            # 构建返回的CompressedImage消息
            compressed_image_msg = CompressedImage()
            compressed_image_msg.format = "jpeg"  # 设置图像格式（如"jpeg"或"png"）
            compressed_image_msg.data = encoded_image.tobytes()

            response.processed_image = compressed_image_msg
            response.success = True
            response.msg.data =  "Success"
            self.get_logger().info(f"Call remove_distortion Success")
            return response

        except Exception as e:
            self.get_logger().error(f"Failed to remove distortion: {str(e)}")
            response.success = False
            response.processed_image = None
            response.msg.data = f"{str(e)}"
            return response

    def generate_semantic_info_callback(self, request, response):
        try:
            
            image_bytes = request.image.data
            camera_name = request.camera_name.data
            caption = request.text.data
            l2w_matrix = odometry_to_transform(request.odometry)
            frame_point_cloud = extract_point_cloud_to_nparray(request.point_cloud) 

            self.get_logger().info(f"Start gen semantic info for: {camera_name}")
            # 先过检测获得简单的语义信息
            res = object_detection_predict(
                image_bytes, 
                caption
            )
            semactic_info_list = {"data": []}
            # 点云映射计算目标的世界坐标
            for od_res in res['data']:
                
                obj_label = od_res['label']
                obj_bbox = od_res['bounding_box']
                
                obj_position, point_confidence = self.pf.get_valid_obj_position(
                    image=image_bytes,
                    camera_name=camera_name,
                    point_cloud=frame_point_cloud,
                    l2w_matrix=l2w_matrix,
                    obj_bbox=obj_bbox
                )
                
                obj_confidence = point_confidence * od_res['confidence'] if point_confidence == 1 else random.uniform(0.2, 0.3)
                obj_position  = obj_position.flatten().tolist()
                self.get_logger().info(f"[Get OBJ] Label: {obj_label}, Position: {obj_position}, Obj Confidence: {obj_confidence}")
                semantic_info = {
                    "caption": obj_label,
                    "position": obj_position,
                    "image_entity": encode_image_bytes_to_base64(image_bytes),
                    "camera_name": camera_name,
                    "confidence": obj_confidence
                }
                semactic_info_list["data"].append(semantic_info)

            response.msg.data = "success"
            response.success = True
            response.data.data = serialize_response(semactic_info_list)

            self.get_logger().info(f"Get Semantic Info")
            return response
                
        except Exception as e:
            self.get_logger().error(f"Failed to generate semantic info: {str(e)}")
            response.msg.data = f"Fail for: {str(e)}"
            response.success = False
            response.data.data = "{}"
            return response
   

def main(args=None):
    rclpy.init(args=args)
    # 创建配置管理器实例
    config_manager = ConfigManager()
    # 加载metacam配置
    metacam_config = config_manager.get_config(
        filename = CONFIG_PATH
    )
    node = MetacamService(metacam_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

