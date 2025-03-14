'''
    接收图像、点云、里程计数据
'''

import os
import cv2
import time
import rclpy
import shutil
import numpy as np


from rclpy.node import Node
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import CompressedImage, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer

from common_utils import ConfigManager
from hub_interface.msg import SensoryData
from .service.skyland_service import SkylandCameraAPI
from .utils.utils import gen_uuid
from .utils.odometry_utils import odometry_to_transform
from .utils.pc_utils import extract_point_cloud_to_nparray
from . import CONFIG_PATH


class MetacamDataReceiver(Node):
    """
    A ROS2 node that subscribes to synchronized image, point cloud, and odometry data, 
    processes the incoming messages, and saves the data to a specified directory.
    """

    # ==================== Initialization ====================
    def __init__(self, configs):
        super().__init__('metacam_data_receiver')
        self.configs = configs
        self.save_raw_data = configs["skyland_metacam"].get("save_camera_data")
        self.init_directories(os.path.join(configs['system'].get("data_path"), configs['system'].get("project_name")))
        self.init_variables()
        self.create_subscribers()
        self.create_publishers()
        self.setup_time_synchronizer()
        self.get_logger().info("MetacamDataReceiver Node Initialized")

    def init_directories(self, save_directory):
        """
        Initialize and clean up the save directory.
        """
        self.save_directory = save_directory or "./saved_data"

        if os.path.exists(self.save_directory):
            shutil.rmtree(self.save_directory)
            self.get_logger().info(f"Old data directory removed: {self.save_directory}")

        os.makedirs(self.save_directory, exist_ok=True)
        self.get_logger().info(f"Data directory created: {self.save_directory}")

    def init_variables(self):
        """
        Initialize internal state variables.
        """
        self.data_receive_count = 0
        self.last_save_time = None
        self.save_interval = 0.1  # Minimum interval between data saves

    def create_subscribers(self):
        """
        Create message subscribers.
        """
        self.left_sub = Subscriber(self, CompressedImage, '/tower/camera/left/jpeg')
        self.right_sub = Subscriber(self, CompressedImage, '/tower/camera/right/jpeg')
        self.pointcloud_sub = Subscriber(self, PointCloud2, '/tower/mapping/cloud_colored')
        self.odometry_sub = Subscriber(self, Odometry, '/tower/mapping/odometry')

    def create_publishers(self):
        """
        Create message publishers.
        """
        self.count_publisher = self.create_publisher(Int32, '/gs_hub/data_receive_count', 10)
        self.processedId_publisher = self.create_publisher(String, '/gs_hub/processed_filename', 10)
        self.sensory_data_publisher = self.create_publisher(SensoryData, '/gs_hub/sensory_data', 10)
    
    def setup_time_synchronizer(self):
        """
        Set up message synchronization for the subscribed topics.
        """
        self.ts = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub, self.pointcloud_sub, self.odometry_sub],
            queue_size=2,
            slop=0.2
        )
        self.ts.registerCallback(self.callback)

    # ==================== Callback Function ====================
    def callback(self, left_msg, right_msg, pointcloud_msg, odometry_msg):
        
        try:
            data_id = gen_uuid()
            self.publish_sensory_data_messages(left_msg, right_msg, pointcloud_msg, odometry_msg)
            self.publish_messages(data_id)
            self.get_logger().info(f"Data send: {data_id}")
            current_time = time.time()
            # 控制接收频率
            if self.save_raw_data and (self.last_save_time is None or (current_time - self.last_save_time) >= 0.1):
                self.last_save_time = current_time
                self.process_and_save_data(left_msg, right_msg, pointcloud_msg, odometry_msg, data_id)

        except Exception as e:
            self.get_logger().error(f'处理数据时发生错误: {e}')

    def process_and_save_data(self, 
        left_msg, 
        right_msg, 
        pointcloud_msg, 
        odometry_msg,
        data_id
    ):
        """
        Process and save incoming messages (images, point cloud, and odometry).
        """
        
        file_paths = self.construct_file_paths(data_id)

        # Process and save images
        self.save_image(file_paths["left_image"], left_msg.data)
        self.save_image(file_paths["right_image"], right_msg.data)

        # Process and save point cloud
        points_array = self.extract_point_cloud(pointcloud_msg)
        # print("points shape", points_array.shape)
        np.save(file_paths["pointcloud"], points_array)

        # Process and save odometry
        transform_matrix = self.odometry_to_T(odometry_msg)
        np.save(file_paths["odometry"], transform_matrix)

        self.get_logger().info(f"Data saved: {data_id}")

        # Stop after saving 100,000 data sets
        if self.data_receive_count >= 100000:
            self.get_logger().info("100,000 data sets saved. Shutting down.")
            self.destroy_node()

    # 节点销毁
    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info('数据接收节点已销毁')


     # ==================== Helper Methods ====================
    
    def construct_file_paths(self, data_id):
        """
        Construct file paths for saving data.
        """
        return {
            "left_image": os.path.join(self.save_directory, f"image_{data_id}_left.jpg"),
            "right_image": os.path.join(self.save_directory, f"image_{data_id}_right.jpg"),
            "pointcloud": os.path.join(self.save_directory, f"points_{data_id}.npy"),
            "odometry": os.path.join(self.save_directory, f"odometry_{data_id}.npy")
        }
    
    def save_image(self, file_path, image_data):
        """
        Decode and save image data.
        """
        np_arr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite(file_path, image)

    def extract_point_cloud(self, pointcloud_msg: PointCloud2) -> np.array:
        """
        Extract points from a point cloud message.
        """
        return extract_point_cloud_to_nparray(pointcloud_msg)

    def odometry_to_T(self, odometry_msg):
        """
        Convert odometry message to a 4x4 transformation matrix.
        """
    
        return odometry_to_transform(odometry_msg)
    
    def publish_messages(self, file_id):
        """
        Publish the current data receive count.
        """
        self.data_receive_count += 1
        count_msg = Int32()
        count_msg.data = self.data_receive_count

        file_id_msg = String()
        file_id_msg.data = file_id

        self.count_publisher.publish(count_msg)
        self.processedId_publisher.publish(file_id_msg)
    
    def publish_sensory_data_messages(self, 
        left_msg, 
        right_msg, 
        pointcloud_msg, 
        odometry_msg
    ):
        sensory_data_msg = SensoryData()

        # 将接收到的消息赋值给 SensoryData
        sensory_data_msg.left_image = left_msg
        sensory_data_msg.right_image = right_msg
        sensory_data_msg.point_cloud = pointcloud_msg
        sensory_data_msg.odometry = odometry_msg

        # 发布 SensoryData 消息
        self.sensory_data_publisher.publish(sensory_data_msg)

    
def main(args=None):
    rclpy.init(args=args)
    
    # 创建配置管理器实例
    config_manager = ConfigManager()
    # 加载metacam配置
    metacam_config = config_manager.get_config(
        filename = CONFIG_PATH, 
    )

    print(metacam_config)

    sky = SkylandCameraAPI(
        metacam_ip=metacam_config["skyland_metacam"]['metacam_ip'],
    )


    sky.set_domain_id()     #设置domain_id
    sky.set_capture_mode()  #设置相机采样
    sky.set_save_mode()     #设置相机帧率
    sky.start_recording()   #开始建图
    

    node = MetacamDataReceiver(metacam_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(sky.stop_recording()) #停止建图
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
