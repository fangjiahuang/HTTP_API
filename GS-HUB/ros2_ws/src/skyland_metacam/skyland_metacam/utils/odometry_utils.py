import numpy as np


from nav_msgs.msg import Odometry


# 四元数转为旋转矩阵
def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw])
    q_norm = q / np.linalg.norm(q)
    qx, qy, qz, qw = q_norm

    # 计算旋转矩阵
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ])
    return R

def odometry_to_transform(odometry_msg: Odometry):
    """
    Convert odometry message to a 4x4 transformation matrix.
    """
    position = odometry_msg.pose.pose.position
    orientation = odometry_msg.pose.pose.orientation
    R = quaternion_to_rotation_matrix(orientation.x, orientation.y, orientation.z, orientation.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [position.x, position.y, position.z]
    return T
