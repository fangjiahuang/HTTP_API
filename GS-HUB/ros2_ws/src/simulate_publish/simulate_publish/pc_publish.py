'''
    模拟发送点云
'''

import os
import rclpy
import numpy as np
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

# 生成点云信息
def create_cloud(points):
    header = std_msgs.msg.Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    point_cloud_data = points[:, :-1]
    point_cloud = point_cloud2.create_cloud(header, fields, point_cloud_data)
    return point_cloud

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/tower/mapping/cloud_colored', 10)
        self.timer = self.create_timer(0.2, self.publish_point_cloud)
        self.count = 0
    # 发布点云信息
    def publish_point_cloud(self):
        pc_file = f"/home/gs/workspace/projects/GS-HUB/ros2_ws/src/simulate_publish/doc/valid_data/points_{3100+self.count:04}.npy"
        points = np.load(pc_file, allow_pickle=True)
        point_clouds = create_cloud(points)
        self.publisher_.publish(point_clouds)
        # self.get_logger().info(f'Published PointCloud2 : {self.count:04}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
