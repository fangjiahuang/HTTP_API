
import numpy as np

from typing import List
from io import BytesIO

from common_utils import CoordinateConverter


class PointsFusion():
    def __init__(self, metacam_obj):
        self.metacam_obj = metacam_obj
        self.coordinate_manipulator = CoordinateConverter(self.metacam_obj)

    def __filter_points(self, points, width_range, height_range):
        cond1 = (points[:, 0] > width_range[0]) & (points[:, 0] < width_range[1])
        cond2 = (points[:, 1] > height_range[0]) & (points[:, 1] < height_range[1])
        cond3 = points[:, 2] > 0
        return points[cond1 & cond2 & cond3]

    def __depth_evaluate(self, points):
        # 深度信息的容错处理
        # 解算距离，以点云z平均值作为z距离
        depth = 1
        z_values = [point[2] for point in points if len(point) == 3]
        if len(z_values) == 0:
            return depth
        
        depth = sum(z_values) / len(z_values)  
        
        if depth == 0:
            depth = 1

        return depth
    
    def __distance_confidence(self, points ,depth):
        return int(bool(abs(points[2] - 1) >= 0.0001))

    def filter_valid_pixel_points(self,
        pixel_points,
        obj_bbox
    ):
        # 缩小检测框
        cond1 = [obj_bbox[0] + (obj_bbox[2] - obj_bbox[0]) * 0.2, obj_bbox[2] - (obj_bbox[2] - obj_bbox[0]) * 0.2]
        cond2 = [obj_bbox[1] + (obj_bbox[3] - obj_bbox[1]) * 0.2, obj_bbox[3] - (obj_bbox[3] - obj_bbox[1]) * 0.2]

        # 过滤点集
        valid_pixel_points = self.__filter_points(pixel_points, cond1, cond2)

        return valid_pixel_points
    

    def get_valid_obj_position(
        self,
        image: BytesIO,
        camera_name: str,
        point_cloud: np.array,
        l2w_matrix: np.array,
        obj_bbox: List
    ):
        pixel_points = self.coordinate_manipulator.coordinate_convert_world_to_pixel(
            l2w_matrix=l2w_matrix,
            points=point_cloud,
            camera_name=camera_name
        )

        # 物体所覆盖的点云
        valid_pixel_points = self.filter_valid_pixel_points(pixel_points, obj_bbox)

        # 估算深度
        depth = self.__depth_evaluate(valid_pixel_points)

        cx = obj_bbox[0] + obj_bbox[2] / 2.0  
        cy = obj_bbox[1] + obj_bbox[3] / 2.0
        bbox_centre = [cx, cy]
        bbox_centre_array = np.array([bbox_centre], dtype=np.float32) 

        image_point_centre = self.coordinate_manipulator.coordinate_convert_pixel_to_image(bbox_centre_array)

        camera_points_centre = self.coordinate_manipulator.coordinate_convert_image_to_camera(
            image_point_centre,
            depth
        )

        point_confidence = self.__distance_confidence(camera_points_centre.flatten().tolist(), depth)

        world_position = self.coordinate_manipulator.coordinate_convert_camera_to_world(
            l2w_matrix=l2w_matrix,
            points = camera_points_centre,
            camera_name=camera_name
        )
        
        return world_position, point_confidence




