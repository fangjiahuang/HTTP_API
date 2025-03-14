
import numpy as np

from typing import List

"""
1.	像素坐标系 是图像数据的基础坐标系，直接与图像数据关联。
2.	图片坐标系 是像素坐标系的物理映射，考虑了相机内参。
3.	相机坐标系 是三维空间的描述，与图片坐标通过投影和相机内参联系。
4.	世界坐标系 是场景的全局三维描述，可以通过相机外参转换到相机坐标系。

一个常见的坐标系转换流程是：
世界坐标系 → 相机坐标系 → 图片坐标系 → 像素坐标系
"""
class CoordinateConverter():
    def __init__(self, metacam):
        self.cam = metacam
        self.image_width, self.image_height = 1600, 1600
        self.fovx_degree, self.fovy_degree = 90, 90
        # 理想内参矩阵
        self.f_x = self.image_width / (2 * np.tan(np.radians(self.fovx_degree) / 2.))
        self.f_y = self.image_height / (2 * np.tan(np.radians(self.fovy_degree) / 2.))
        self.c_x = self.image_width / 2.
        self.c_y = self.image_height / 2.

        self.intrinsic =np.array([
            [self.f_x, 0, self.c_x],
            [0, self.f_y, self.c_y],
            [0, 0, 1]
        ])
        # 内参矩阵的逆
        self.inverse_intrinsic = np.linalg.inv(self.intrinsic)

    # 世界坐标系 → 像素坐标系，第三个为z深度
    def coordinate_convert_world_to_pixel(self, l2w_matrix, points, camera_name):
        # print(type(l2w_matrix), l2w_matrix.shape, l2w_matrix)
        # print(type(camera_name), camera_name)
        
        # 世界到相机坐标变换矩阵
        w2c = self.cam[camera_name].get_w2c(l2w_matrix)

        # print("w2c shape:", w2c.shape)  # (4, 4)
        # print("points.T shape:", points.T.shape)  # (4, n)

        # 解算相机坐标系坐标
        transformed_data = (w2c @ points.T).T
        transformed_data = transformed_data[:, :3]
        # 将前两位解算为像素坐标
        pixel_points = self.cam[camera_name].project_camera_points_pinhole(
            transformed_data, 
            self.image_width, 
            self.image_height,
            self.fovx_degree, 
            self.fovy_degree
        )
        return pixel_points

    # 像素坐标系 → 世界坐标系
    def coordinate_convert_pixel_to_world(self, l2w_matrix, points, camera_name):
        ...

    # 像素坐标系 → 图片坐标系
    def coordinate_convert_pixel_to_image(self, pixel_points: np.ndarray):
        """
        像素坐标系 → 图片坐标系
        :param pixel_points: N×2 的像素坐标数组，每行 [u, v]
        :return: N×2 的图片坐标系点，每行 [x', y']
        """

        # 1. 构造齐次坐标：将 [u, v] -> [u, v, 1]
        #   pixel_points.shape = (N, 2)
        #   ones.shape         = (N, 1)
        #   pixel_homogeneous  = (N, 3)
        ones = np.ones((pixel_points.shape[0], 1), dtype=pixel_points.dtype)
        pixel_homogeneous = np.hstack([pixel_points, ones])
        
        # 2. 通过内参逆矩阵将像素坐标系转换到图片坐标系 (齐次)
        #    image_points_hom.shape = (N, 3)
        image_points_hom = (self.inverse_intrinsic @ pixel_homogeneous.T).T

        # 3. 齐次坐标归一化： [x', y', w'] -> [x'/w', y'/w']
        #    取前2维 / 第3维，得到 N×2
        w = image_points_hom[:, 2:3]  # shape (N, 1)
        # 防止除 0，可检查 w 是否接近 0
        image_points = image_points_hom[:, :2] / w

        return image_points

    # 图片坐标系 → 相机坐标系
    def coordinate_convert_image_to_camera(self, image_points, depths: float):
        """
        图片坐标系 → 相机坐标系 (批量处理)
        :param image_points: (N, 2) 的图片坐标系点集，每行 [x', y']
        :param depths:       (N,)   的深度数组，每个值对应相机坐标系下的 Z_c
        :return:             (N, 3) 的相机坐标系点集，每行 [X_c, Y_c, Z_c]
        """
        # 如果 depths 是标量，将其转为形状 (1,)
        if np.isscalar(depths):
            depths = np.array([depths], dtype=image_points.dtype)
        elif isinstance(depths, np.ndarray) and depths.ndim == 0:
            depths = depths.reshape(1)

        #    image_points_hom 形状 (N, 3)
        ones = np.ones((image_points.shape[0], 1), dtype=image_points.dtype)
        image_points_hom = np.hstack([image_points, ones])

        # 按深度进行缩放
        # 形状 (N, 3) × (N,1) -> (N, 3), 广播机制
        camera_points = image_points_hom * depths[:, None]
        
        # 返回结果，每行 [X_c, Y_c, Z_c]
        return camera_points

    # 相机坐标系 → 世界坐标系
    def coordinate_convert_camera_to_world(self, l2w_matrix, points, camera_name):
        # 解算相机到世界坐标转换矩阵
        c2w = self.cam[camera_name].get_c2w(l2w_matrix)
        # 齐次变为四维
        ones = np.ones((points.shape[0], 1))
        homogeneous_points = np.hstack([points, ones])
        # 将相机坐标转换为世界坐标
        return (c2w @ homogeneous_points.T).T
