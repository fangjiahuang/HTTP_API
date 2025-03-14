# 导入 FastAPI 的路由模块，用于定义一组相关的接口
from fastapi import APIRouter, HTTPException, UploadFile, File
# 导入 Pydantic 的 BaseModel，用于定义数据模型，验证和解析输入数据
from pydantic import BaseModel
# 导入 base64 模块，用于处理 Base64 编码和解码
import base64
# 从 object_detection_service 模块导入目标检测预测函数
from skyland_metacam.service.object_detection_service import object_detection_predict

# 创建一个 APIRouter 实例，定义路由前缀为 "/v1/object_detection"，并添加标签 "object_detection"
router = APIRouter(prefix="/v1/object_detection", tags=["object_detection"])

# 定义一个数据模型，用于接收目标检测接口的输入数据
class ObjectDetectionRequest(BaseModel):
    """
    请求模型，用于接收目标检测的输入数据。
    """
    text: str  # 目标文本，表示检测的目标内容
    image: str = None  # 图像的 Base64 编码（可选，用于直接传入 Base64 图像）
    image_file: UploadFile = None  # 图像文件（可选，用于上传图片文件）

# 定义一个目标检测接口，路径为 "/predict"，支持 POST 请求
@router.post("/predict")
async def object_detection_endpoint(request: ObjectDetectionRequest):
    """
    目标检测接口。

    支持两种方式传入图像：
    1. 直接传入 Base64 编码的图像字符串（`image` 字段）。
    2. 上传图像文件（`image_file` 字段）。

    Args:
        request (ObjectDetectionRequest): 请求数据，包含目标文本和图像。

    Returns:
        dict: 目标检测的结果。
    """
    try:
        # 获取图像数据
        if request.image_file:  # 如果用户上传了文件
            # 读取上传的文件内容，返回字节流
            image_bytes = await request.image_file.read()
        elif request.image:  # 如果用户传入了 Base64 编码的图像
            # 将 Base64 编码的字符串解码为字节流
            image_bytes = base64.b64decode(request.image)
        else:
            # 如果既没有上传文件，也没有提供 Base64 图像，抛出 400 错误
            raise HTTPException(status_code=400, detail="Either 'image' or 'image_file' must be provided.")

        # 调用目标检测服务，传入图像字节流和目标文本
        response = object_detection_predict(image_bytes, request.text)

        # 检查目标检测服务返回的结果是否包含错误
        if response.get("status") == "error":
            # 如果服务返回了错误信息，抛出 500 错误
            raise HTTPException(status_code=500, detail=response.get("message", "Unknown error occurred."))

        # 如果服务返回成功，将结果封装后返回给客户端
        return {"code": 0, "msg": "success", "data": response}

    except HTTPException as e:
        # 如果捕获到 HTTPException 异常，直接重新抛出
        raise e
    except Exception as e:
        # 如果捕获到其他异常，抛出 500 错误，并返回错误信息
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")
