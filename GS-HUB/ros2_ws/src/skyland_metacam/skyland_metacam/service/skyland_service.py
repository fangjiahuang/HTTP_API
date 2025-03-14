
'''
    与感知塔的通信
'''
import requests
from common_utils import ConfigManager

class SkylandCameraAPI:

    def __init__(self, metacam_ip):
        self.base_url = f"http://{metacam_ip}:8000/api/v1"

    def post(self, endpoint, data):
        url = f"{self.base_url}{endpoint}"
        try:
            response = requests.post(url, json=data)
            print(response.json())
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error posting to {url}: {e}")

    def get(self, endpoint):
        url = f"{self.base_url}{endpoint}"
        try:
            response = requests.get(url)
            print(response.json())
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error getting from {url}: {e}")

    def set_capture_mode(self, image_quality=75, resolution_division=2, in_rosbag=True):
        data = {
            "image_quality": image_quality,
            "resolution_division": resolution_division,
            "in_rosbag": in_rosbag
        }
        return self.post("/camera/set_capture_mode", data)

    def set_save_mode(self, save_interval=1, distance_moved=0, angle_rotated=0, blur_score=0):
        data = {
            "save_interval": save_interval,
            "distance_moved": distance_moved,
            "angle_rotated": angle_rotated,
            "blur_score": blur_score
        }
        return self.post("/camera/set_save_mode", data)

    def start_recording(self):
        return self.get("/debug/fast_request_start_record")

    def stop_recording(self):
        return self.get("/debug/fast_request_stop_record")

    def set_domain_id(self, domain_id=42):
        url = "http://192.168.19.97/api/v1/ros2/set_domain_id" 
        data = {
                    "domain_id": 42
                }
        response = requests.post(url, json = data)
        print(response.json())

    def set_nav_config(self, world_point, mode=0, parameters=[1]):
        data = {
            "mode": mode,
            "parameters": parameters,
            "points": [{
                "x": world_point[0],
                "y": world_point[1],
                "z": world_point[2]
            }]
        }
        print(data)
        return self.post("/settings/set_nav_config", data)

    def start_navigation(self):
        return self.get("/nav/request_start")

    def stop_navigation(self):
        return self.get("/nav/request_stop")

# Example usage
if __name__ == "__main__":
    # camera_api = SkylandCameraAPI(enable_requests=config.request2skyland)

    # # Example calls
    # camera_api.set_capture_mode()
    # camera_api.set_save_mode()
    # camera_api.start_recording()
    # camera_api.set_domain_id(domain_id=42)
    # camera_api.stop_recording()
    # camera_api.set_nav_config([1.0, 2.0, 3.0])
    # camera_api.start_navigation()
    # camera_api.stop_navigation()
    ...