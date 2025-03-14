# image_api.py

# 导入 FastAPI 框架，用于创建 API
from fastapi import FastAPI, File, UploadFile, HTTPException
# 导入 Pydantic 的 BaseModel，用于定义请求的数据结构
from pydantic import BaseModel
# 导入 os 模块，用于文件和目录操作
import os
# 导入 json 模块，用于处理 JSON 数据
import json
# 导入 cv2 模块，作为 OpenCV 的接口，用于图像处理
import cv2 as cv
# 导入 numpy 模块，用于数值计算
import np
# 从 typing 模块导入 List，用于类型注解
from typing import List
# 导入自定义的 common_utils 模块中的 metacam_sdk 和 timer
from common_utils import metacam_sdk, timer

# 创建一个 FastAPI 应用实例，名称为 "Image Undistortion API"
app = FastAPI(title="Image Undistortion API")

# 定义一个请求的数据模型，用于接收客户端发送的 JSON 数据
class UndistortionRequest(BaseModel):
    # 图片的唯一标识符
    id: int
    # 相机的名称，用于匹配对应的相机参数
    camera_name: str
    # 存放输入图片的文件夹路径
    input_folder: str
    # 包含相机参数的 JSON 数据
    data: dict  # 包含相机参数的 JSON 数据

# 加载相机数据的函数，与 image_utils 中的功能相同
def load_camera_data(json_path):
    """
    加载并返回相机数据和相机参数。
    
    参数:
        json_path (str): JSON 文件的路径。
        
    返回:
        data: 从 JSON 文件加载的数据。
        cams: 从 JSON 文件读取的相机参数。
    """
    # 打开并读取 JSON 文件
    with open(json_path, 'r') as file:
        data = json.load(file)
    # 使用 metacam_sdk 从 JSON 文件中读取相机参数
    cams = metacam_sdk.read_lidar_camera_from_json(json_path)
    return data, cams

# 提取相机内参的函数，与 image_utils 中的功能相同
def extract_camera_intrinsics(camera_data):
    """
    从相机数据中提取内参矩阵 K，畸变系数 D 和图像尺寸 size。
    
    参数:
        camera_data (dict): 包含相机内参和畸变参数的字典。
        
    返回:
        K (np.ndarray): 内参矩阵。
        D (np.ndarray): 畸变系数。
        size (tuple): 图像的宽度和高度。
    """
    # 提取焦距 fx 和 fy
    fx = camera_data['intrinsic']['fl_x']
    fy = camera_data['intrinsic']['fl_y']
    # 提取主点坐标 cx 和 cy
    cx = camera_data['intrinsic']['cx']
    cy = camera_data['intrinsic']['cy']
    # 假设校准图像的宽度和高度为 1600x1600
    width_calib, height_calib = 1600, 1600
    # 构建内参矩阵 K
    K = np.array([[fx / 2, 0, cx / 2],
                  [0, fy / 2, cy / 2],
                  [0, 0, 1]], dtype=np.float64)
    
    # 提取畸变参数 k1, k2, k3, k4
    k1 = camera_data['distortion']['params']['k1']
    k2 = camera_data['distortion']['params']['k2']
    k3 = camera_data['distortion']['params']['k3']
    k4 = camera_data['distortion']['params']['k4']
    # 构建畸变系数数组 D
    D = np.array([k1, k2, k3, k4], dtype=np.float64)
    
    # 定义图像尺寸
    size = (width_calib, height_calib)
    return K, D, size

# 去除鱼眼镜头的畸变函数，与 image_utils 中的功能相同
def remove_fisheyes(camera_data, img_obj):
    """
    使用相机内参和畸变系数去除鱼眼畸变。
    
    参数:
        camera_data (dict): 包含相机内参和畸变参数的字典。
        img_obj (np.ndarray): 输入的畸变图像。
        
    返回:
        undistorted_image (np.ndarray): 去畸变后的图像。
    """
    # 提取相机内参和畸变参数
    K, D, size = extract_camera_intrinsics(camera_data)
    # 初始化去畸变的映射
    map1, map2 = cv.fisheye.initUndistortRectifyMap(
        K=K, D=D, R=None, P=K, size=size, m1type=cv.CV_32FC1)
    # 应用映射进行去畸变
    undistorted_image = cv.remap(img_obj, map1, map2, interpolation=cv.INTER_LINEAR)
    return undistorted_image

# 定义一个内部函数，用于去畸变图像
def undistort_image(id: int, camera_name: str, input_folder: str, camera_data: dict) -> np.ndarray:
    """
    根据提供的参数去畸变图像。
    
    参数:
        id (int): 图片的唯一标识符。
        camera_name (str): 相机的名称。
        input_folder (str): 输入图片所在的文件夹路径。
        camera_data (dict): 包含相机内参和畸变参数的字典。
        
    返回:
        undistorted_image (np.ndarray): 去畸变后的图像。
    """
    # 构造图片的文件名
    filename = f"image_{id}_{camera_name}.jpg"
    # 拼接图片的完整路径
    img_path = os.path.join(input_folder, filename)
    # 读取图片
    img = cv.imread(img_path)
    # 如果图片未找到，抛出 HTTP 404 错误
    if img is None:
        raise HTTPException(status_code=404, detail=f"无法读取图片: {filename}")
    
    # 调用去除畸变的函数
    undistorted_image = remove_fisheyes(camera_data, img)
    return undistorted_image

# 定义一个 POST 请求的端点 /undistort/
@app.post("/undistort/")
async def undistort_image_endpoint(request: UndistortionRequest):
    """
    去畸变图片的 API 端点。
    
    接收包含图片ID、相机名称、输入文件夹路径和相机数据的 JSON 请求，
    返回去畸变后的图片。
    """
    try:
        # 调用去畸变函数处理图像
        undistorted = undistort_image(
            request.id,
            request.camera_name,
            request.input_folder,
            request.data
        )
        # 将 NumPy 数组编码为 JPEG 图像的字节流
        _, img_encoded = cv.imencode('.jpg', undistorted)
        # 将编码后的图像转换为字节类型
        img_bytes = img_encoded.tobytes()
        
        # 返回去畸变后的图片作为响应，设置媒体类型为 image/jpeg
        return Response(content=img_bytes, media_type="image/jpeg")
        
        # 如果你希望返回 Base64 编码的字符串，可以使用以下代码：
        # base64_str = encode_image_bytes_to_base64(img_bytes)
        # return {"undistorted_image": base64_str}
        
    except Exception as e:
        # 如果发生任何异常，返回 HTTP 500 错误，并返回错误信息
        raise HTTPException(status_code=500, detail=str(e))

# 定义另一个 POST 请求的端点 /undistort_with_file/
@app.post("/undistort_with_file/")
async def undistort_image_with_file_endpoint(
    id: int = Form(...),
    camera_name: str = Form(...),
    input_folder: str = Form(...),
    camera_data_file: UploadFile = File(...)
):
    """
    去畸变图片的 API 端点，支持上传相机参数文件。
    
    接收图片ID、相机名称、输入文件夹路径和相机参数文件的上传，
    返回去畸变后的图片。
    """
    try:
        # 读取上传的相机参数文件内容
        camera_data_json = await camera_data_file.read()
        # 将 JSON 字符串解析为字典
        camera_data = json.loads(camera_data_json)
        
        # 调用去畸变函数处理图像
        undistorted = undistort_image(id, camera_name, input_folder, camera_data)
        # 将 NumPy 数组编码为 JPEG 图像的字节流
        _, img_encoded = cv.imencode('.jpg', undistorted)
        # 将编码后的图像转换为字节类型
        img_bytes = img_encoded.tobytes()
        
        # 返回去畸变后的图片作为响应，设置媒体类型为 image/jpeg
        return Response(content=img_bytes, media_type="image/jpeg")
        
    except FileNotFoundError:
        # 如果相机参数文件或图片未找到，返回 HTTP 404 错误
        raise HTTPException(status_code=404, detail="相机参数文件或图片未找到。")
    except json.JSONDecodeError:
        # 如果相机参数文件格式不正确，返回 HTTP 400 错误
        raise HTTPException(status_code=400, detail="相机参数文件格式不正确。")
    except Exception as e:
        # 如果发生其他异常，返回 HTTP 500 错误，并返回错误信息
        raise HTTPException(status_code=500, detail=str(e))

# 如果直接运行此脚本，则启动 FastAPI 服务器
if __name__ == "__main__":
    # 导入 uvicorn，用于运行 ASGI 服务器
    import uvicorn
    # 启动 FastAPI 应用，监听所有 IP 地址的 8000 端口，并开启调试模式
    uvicorn.run(app, host="0.0.0.0", port=8000)