
import sys
import json
import requests
import numpy as np
from config import config
from typing import List, Union


# 文本编码
def text_encoder(model_url, data: Union[List[str], str]):
    payload = {"sentences": data}
    response = requests.post(model_url, json=payload)

    if response.status_code != 200:
        raise ValueError(f"请求失败，状态码: {response.status_code}")
    response = json.loads(response.text)
    emb = response["data"]["embeddings"]
    return emb


if __name__ == "__main__":
    url = "http://127.0.0.1:36222/embed/"  # Adjust the URL if running elsewhere

    # Define the sentences to send in the request
    # data = ["hello", "goodbye", "多问一句 这个路径是自动规划的还是可以手动设"]
    data = "hello"
    print(text_encoder(url, data))