
import numpy as np
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import CompressedImage, PointCloud2

def extract_point_cloud_to_nparray(pointcloud_msg: PointCloud2) -> np.array:
    """
    Extract points from a point cloud message.
    """
    points = point_cloud2.read_points_list(
        pointcloud_msg, 
        field_names=("x", "y", "z"), 
        skip_nans=True
    )

    return np.array([[p.x, p.y, p.z, 1] for p in points], dtype=np.float32)