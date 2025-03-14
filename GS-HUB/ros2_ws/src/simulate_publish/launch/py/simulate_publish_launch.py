from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    left_publish = Node(package="simulate_publish", executable="left_img_publish", name="left_publish")
    right_publish = Node(package="simulate_publish", executable="right_img_publish", name="right_publish")
    pc_publish = Node(package="simulate_publish", executable="pc_publish", name="pc_publish")
    odom_publish = Node(package="simulate_publish", executable="odom_publish", name="odom_publish")
    return LaunchDescription([left_publish, right_publish, pc_publish, odom_publish])
