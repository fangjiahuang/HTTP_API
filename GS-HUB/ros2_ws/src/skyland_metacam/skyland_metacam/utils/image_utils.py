'''
    输入原始图片，输出去畸变图片，图片处理的第一步
'''

import os
import json
import cv2 as cv
import numpy as np
import base64

#尝试用sys.path.append加入metacam_sdk
#import sys
#sys.path.append("/home/fangjiahuang/projects/GS-HUB/ros2_ws/src/common_utils/common_utils/meta-cam-sdk/build/py")
#import metacam_sdk
#from common_utils import timer
from common_utils import metacam_sdk, timer

# 加载 JSON 文件并返回相机数据
def load_camera_data(json_path):
    with open(json_path, 'r') as file:
        data = json.load(file)
    cams = metacam_sdk.read_lidar_camera_from_json(json_path)
    return data, cams

# 去除鱼眼畸变
def remove_fisheyes(camera_data, img_obj):
    # 提取相机内参
    K, D, size = extract_camera_intrinsics(camera_data)
    # 生成去畸变映射
    map1, map2 = cv.fisheye.initUndistortRectifyMap(
        K=K, D=D, R=None, P=K, size=size, m1type=cv.CV_32FC1)
    # 应用映射进行去畸变
    undistorted_image = cv.remap(img_obj, map1, map2, interpolation=cv.INTER_LINEAR)
    return undistorted_image

# 输入相机文件，输出提取的K，D，size
def extract_camera_intrinsics(camera_data):
    # 提取相机内参和畸变参数
    fx = camera_data['intrinsic']['fl_x']
    fy = camera_data['intrinsic']['fl_y']
    cx = camera_data['intrinsic']['cx']
    cy = camera_data['intrinsic']['cy']
    width_calib, height_calib = 1600, 1600
    K = np.array([[fx / 2, 0, cx / 2],
                  [0, fy / 2, cy / 2],
                  [0, 0, 1]], dtype=np.float64)
    
    k1 = camera_data['distortion']['params']['k1']
    k2 = camera_data['distortion']['params']['k2']
    k3 = camera_data['distortion']['params']['k3']
    k4 = camera_data['distortion']['params']['k4']
    D = np.array([k1, k2, k3, k4], dtype=np.float64)
    
    size = (width_calib, height_calib)
    return K, D, size

# 图片去畸变
@timer
def undistortion(id, camera_name, input_folder, data, output_folder):
    # 读取图片
    filename = f"image_{id}_{camera_name}.jpg"
    img_path = os.path.join(input_folder, filename)
    img = cv.imread(img_path)
    if img is None:
        print(f"无法读取图片: {filename}")
        return
    # 读取相机参数文件
    camera_data = next(cam for cam in data['cameras'] if cam['name'] == camera_name)
    # 去除畸变
    undistorted_image = remove_fisheyes(camera_data, img)
    # 保存图片
    output_path = os.path.join(output_folder, filename)
    cv.imwrite(output_path, undistorted_image)
    # print(f"去畸变图片已保存为: {filename}")

# 将图像字节流编码为 Base64
def encode_image_bytes_to_base64(image_bytes: bytes) -> str:
    return base64.b64encode(image_bytes).decode("utf-8")

if __name__ == "__main__":
    ...
