
import rclpy
import os
import shutil
import time
import uuid

from rclpy.node import Node
from qdrant_client import QdrantClient
from common_utils import ConfigManager
from queue import Queue

from . import CONFIG_PATH
from .semantic_controller.semantic_manager import SemanticManger
from hub_interface.msg import SensoryData, SemanticData
from hub_interface.srv import DistortionRemoval, SemanticInfo
from hub_interface.srv import StartMapBuilder, EndMapBuilder, StartGenSemantic, EndGenSemantic
from common_utils import serialize_response, unserialize_response
from .service.text_encoder import text_encoder
from .utils.service_utils import prepare_distortion_request, prepare_semantic_request


class SemanticMapBuilder(Node):

    def __init__(self, configs):
        super().__init__('semantic_map_builder')

        self.qdrant_client = QdrantClient(configs['semantic_map'].get('vecdb_url'))
        self.semantic_manager = SemanticManger(db_client=self.qdrant_client)
        self.dimension = configs['semantic_map'].get('vec_dimension')
        
        self.build_in_progress = False
        self._batch_buffer = {
            "ids": [],
            "vectors": [],
            "payloads": []
        }

        self.__init_topic_suscriber()
        self.__init_db_server()
        self.__init_client()

        # 定时器：每 2s 检查一次是否需要 flush
        self._timer = self.create_timer(2.0, self._flush_batch_callback)

    def __init_topic_suscriber(self):
        # 订阅 /gs_hub/sensory_data 话题
        self.left_semantic_sub = self.create_subscription(
            SemanticData,
            '/gs_hub/semantic_data/left',
            self.semantic_data_callback,
            10
        )

        self.right_semantic_sub = self.create_subscription(
            SemanticData,
            '/gs_hub/semantic_data/right',
            self.semantic_data_callback,
            10
        )

    def __init_db_server(self):
        self.build_db_service = self.create_service(
            srv_type=StartMapBuilder,
            srv_name='/gs_hub/start_build_db',
            callback=self.build_db_callback
        )

        self.end_build_db_service = self.create_service(
            srv_type=EndMapBuilder,
            srv_name='/gs_hub/end_build_db',
            callback=self.end_build_db_callback
        )

    def __init_client(self):
        
         # 创建两个客户端，分别对应 start 和 end 服务
        self._start_client = self.create_client(StartGenSemantic, '/gs_hub/start_gen_semantic')
        self._end_client = self.create_client(EndGenSemantic, '/gs_hub/end_gen_semantic')
        
        self.get_logger().info("Waiting for build services ...")
        self._start_client.wait_for_service()
        self._end_client.wait_for_service()
        self.get_logger().info("Build services are available now!")

    def call_start_service(self):
        """调用 StartMapBuilder 服务"""
        request = StartGenSemantic.Request()

        future = self._start_client.call_async(request)
        future.add_done_callback(self._start_service_done_callback)
    
    def _start_service_done_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                if response.success:
                    self.get_logger().info(f"[Start] Succeeded: {response.msg}")
                else:
                    self.get_logger().error(f"[Start] Failed: {response.msg}")
            else:
                self.get_logger().error("[Start] Service call returned None")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
    
    def call_end_service(self):
        """调用 EndGenSemantic 服务"""
        self.get_logger().info("Ending generation semantic info...")
        request = EndGenSemantic.Request()  # 如果有请求字段也在此进行赋值
        future = self._end_client.call_async(request)
        future.add_done_callback(self._end_service_done_callback)

    def _end_service_done_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                if response.success:
                    self.get_logger().info(f"[End] Succeeded: {response.msg}")
                else:
                    self.get_logger().error(f"[End] Failed: {response.msg}")
            else:
                self.get_logger().error("[End] Service call returned None")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def semantic_data_callback(self, msg: SemanticData):
        if not self.build_in_progress:
            return
        
        self.get_logger().info("receive semantic data")
        id = msg.id.data
        vector = list(msg.vector)
        
        payload = {
            "camera_name": msg.camera_name.data,
            "label": msg.caption.data,
            "position": msg.position,
            "confidence": msg.confidence
        }
        self._batch_buffer['ids'].append(id)
        self._batch_buffer['vectors'].append(vector)
        self._batch_buffer['payloads'].append(payload)

    def build_db_callback(self, request, response):
        """
        开始构建数据库。
        """
        collection_name, dimension = request.collection_name.data, int(request.dimension)
        # 1. 初始化数据库
        ok = self.semantic_manager.init_vecdb(collection_name, dimension)
        if not ok:
            self.get_logger().error(f"Failed to init collection '{collection_name}'")
            response.success = False
            response.msg.data = f"Failed to init collection '{collection_name}'"
            return response
        
        self.build_in_progress = True
        self.call_start_service() # 发布semantic信息
        self.get_logger().info(f"Build started. Collection: {collection_name}, Dimension: {dimension}")

        response.success = True
        response.msg.data = f"Start building database '{collection_name}' with dimension={dimension}."
        return response
    
    def end_build_db_callback(self, request, response):
        """
        结束构建。
        """
        self.call_end_service()
        self.build_in_progress = False
        if len(self._batch_buffer["ids"]) > 0:
            self._flush_batch()
        response.success = True
        response.message = "Build ended successfully."
        return response
    
    def _flush_batch_callback(self):
        """
        这是定时器回调，每秒检查一次是否有未满批次，但依旧需要写入的数据。
        """
        # 如果还在构建阶段且缓存不为空，则写入
        if self.build_in_progress and len(self._batch_buffer["ids"]) > 0:
            self._flush_batch()
        
    def _flush_batch(self):
        """
        将缓存的数据批量插入数据库，然后清空缓存
        """
        self.get_logger().info("start flush")
        ids = self._batch_buffer["ids"]
        vectors = self._batch_buffer["vectors"]
        payloads = self._batch_buffer["payloads"]

        # 执行批量写入
        ok = self.semantic_manager.map_builder(
            ids=ids,
            vectors=vectors,
            payloads=payloads
        )

        if not ok:
            self.get_logger().error("Batch insertion to VecDB failed!")
        else:
            self.get_logger().info(f"Successfully inserted batch of size {len(ids)}.")
        
        # 清空缓存
        self._batch_buffer["ids"].clear()
        self._batch_buffer["vectors"].clear()
        self._batch_buffer["payloads"].clear()


def main():
    rclpy.init()
    node = None
    try:
        config_manager = ConfigManager()
        semantic_config = config_manager.get_config(filename = CONFIG_PATH)

        node = SemanticMapBuilder(semantic_config)
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
