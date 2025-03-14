'''
    模拟发送里程计信息
'''

import os
import rclpy
import numpy as np
import tf_transformations
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/tower/mapping/odometry', 10)
        self.timer = self.create_timer(0.2, self.publish_odometry)
        self.count = 0
    # 发布里程计信息
    def publish_odometry(self):
        odom = Odometry()
        # 填充 header
        odom.header.stamp = rclpy.clock.Clock().now().to_msg()
        odom_file = f"/home/gs/workspace/projects/GS-HUB/ros2_ws/src/simulate_publish/doc/valid_data/odometry_{3100+self.count:04}.npy"
        self.odometry_data = np.load(odom_file, allow_pickle=True)
        # 旋转矩阵转为四元数
        R = self.odometry_data[0:3, 0:3]
        T = np.eye(4)
        T[:3, :3] = R
        quaternion = tf_transformations.quaternion_from_matrix(T)
        # 填充位姿
        odom.pose.pose.position = Point(x = float(self.odometry_data[0][3]), y = float(self.odometry_data[1][3]), z = float(self.odometry_data[2][3]))
        odom.pose.pose.orientation = Quaternion(x = quaternion[0], y = quaternion[1], z = quaternion[2], w = quaternion[3])
        # 发布里程计信息
        self.publisher_.publish(odom)
        # self.get_logger().info(f'Published odom : {self.count:04}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
