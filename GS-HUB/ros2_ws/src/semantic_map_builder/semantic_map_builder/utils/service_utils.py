from hub_interface.srv import DistortionRemoval, SemanticInfo

import time

def prepare_distortion_request(image, camera_name):
    """准备去畸变请求"""
    req = DistortionRemoval.Request()
    req.image = image
    req.camera_name.data = camera_name
    return req


def prepare_semantic_request(image, msg, camera_name):
    """准备语义信息请求"""
    req = SemanticInfo.Request()
    req.image = image
    req.text.data = ""
    req.point_cloud = msg.point_cloud
    req.odometry = msg.odometry
    req.camera_name.data = camera_name
    return req

