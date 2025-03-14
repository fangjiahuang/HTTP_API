import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)).rsplit(os.sep,3)[0])

import requests
import base64

from typing import Any, Dict
from ..utils.image_utils import encode_image_bytes_to_base64

def object_detection_predict(
        image_bytes: bytes, 
        target_text: str, 
        url: str = "http://127.0.0.1:35551/v1/object_detection/predict"
    ) -> Dict[str, Any]:

    """
    调用 目标检测模型

    Args:
        image_bytes (bytes): 图像的字节流
        target_text (str): 目标文本
        url (str): 推理服务的 URL，默认是本地服务

    Returns:
        dict: 响应的 JSON 数据
    """
    # 编码图像为 Base64
    image_base64 = encode_image_bytes_to_base64(image_bytes)

    # 准备请求数据
    request_data = {
        "image": image_base64,
        "text": target_text
    }

    try:
        # 发起 POST 请求
        response = requests.post(url, json=request_data)

        # 检查响应状态
        if response.status_code == 200:
            return response.json()
        else:
            return {
                "status": "error",
                "message": f"Request failed with status code {response.status_code}: {response.text}"
            }
    except Exception as e:
        return {
            "status": "error",
            "message": f"An error occurred: {str(e)}"
        }
