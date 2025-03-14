#!/usr/bin/env python3
# 文件名: test_build_db.py

import rclpy
from rclpy.node import Node
from hub_interface.srv import StartMapBuilder, EndMapBuilder, StartGenSemantic, EndGenSemantic

class BuildDbTester(Node):
    def __init__(self):
        super().__init__('build_db_tester')
        
        # 创建两个客户端，分别对应 start 和 end 服务
        self._start_client = self.create_client(StartGenSemantic, '/gs_hub/start_gen_semantic')
        self._end_client = self.create_client(EndGenSemantic, '/gs_hub/end_gen_semantic')
        
        self.get_logger().info("Waiting for build services ...")
        self._start_client.wait_for_service()
        self._end_client.wait_for_service()
        self.get_logger().info("Build services are available now!")

    def call_start_service(self, collection_name: str, dimension: int):
        """调用 StartMapBuilder 服务"""
        request = StartGenSemantic.Request()


        self.get_logger().info(f"Requesting start build db: {collection_name} (dim={dimension})")
        future = self._start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"[Start] Succeeded: {response.msg}")
            else:
                self.get_logger().error(f"[Start] Failed: {response.msg}")
        else:
            self.get_logger().error("[Start] Service call returned None")

    def call_end_service(self):
        """调用 EndMapBuilder 服务"""
        request = EndGenSemantic.Request()  # 如果有请求字段也在此进行赋值
        
        self.get_logger().info("Requesting end build db...")
        future = self._end_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"[End] Succeeded: {response.msg}")
            else:
                self.get_logger().error(f"[End] Failed: {response.msg}")
        else:
            self.get_logger().error("[End] Service call returned None")


def main(args=None):
    rclpy.init(args=args)

    node = BuildDbTester()

    # 1. 调用开始构建数据库服务
    node.call_start_service(collection_name="zyh", dimension=768)

    # 此时如果你想验证正在收集/写入数据库逻辑，可等待一段时间
    # 或者在此处加入更多的逻辑来检查 node 中是否在正常收集信息
    # node.get_logger().info("Sleeping 5 seconds to let the node build some data ...")
    # node.get_clock().sleep_for(rclpy.duration.Duration(seconds=30))

    # 2. 调用结束构建数据库服务
    # node.call_end_service()

    # 退出
    node.destroy_node()
    rclpy.shutdown()
