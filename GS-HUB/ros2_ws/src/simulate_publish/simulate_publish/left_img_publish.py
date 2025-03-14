'''
    模拟发送左相机图片
'''

import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('left_image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/tower/camera/left/jpeg', 10)
        self.timer = self.create_timer(0.2, self.publish_image)  # 10 Hz
        self.count = 0

    def publish_image(self):
        # 读取图像
        img_path = f"/home/gs/workspace/projects/GS-HUB/ros2_ws/src/simulate_publish/doc/valid_data/image_{3100+self.count:04}_left.jpg"
        img = cv2.imread(img_path)
        if img is None:
            self.get_logger().error("Failed to read image")
            return

        # 将图像转换为 np.uint8
        img_np = np.array(img, dtype=np.uint8)

        # 编码为 JPEG 格式
        _, jpeg_img = cv2.imencode('.jpg', img_np)
        jpeg_bytes = jpeg_img.tobytes()

        # 创建 CompressedImage 消息
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = jpeg_bytes

        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published left image : {self.count:04}")

        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
