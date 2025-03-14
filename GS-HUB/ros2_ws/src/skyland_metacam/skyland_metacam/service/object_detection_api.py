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
    try:
        # 检查是否提供了 image_file
        if request.image_file:
            print("输入格式是.jpg或.png文件")
            image_bytes = await request.image_file.read()  #直接读.jpg(.png)文件，因为它们本身就是二进制，也就是字节流
        elif request.image:
            # 解码 Base64 字符串为字节
            print("输入格式是base64 string")
            try:
                image_bytes = base64.b64decode(request.image, validate=True)  # base64->字节流
            except base64.binascii.Error as e:
                print("解码错误")
                raise HTTPException(status_code=400, detail="Invalid Base64 encoding.")
        else:
            print("输入为空")
            raise HTTPException(status_code=400, detail="Either 'image' or 'image_file' must be provided.")

        # 调用目标检测服务，传入图像字节流和目标文本
        response = object_detection_predict(image_bytes, request.text)

        # 检查目标检测服务返回的结果是否包含错误
        if response.get("status") == "error":
            raise HTTPException(status_code=500, detail=response.get("message", "Unknown error occurred."))

        # 返回成功结果
        return {"code": 0, "msg": "success", "data": response}

    except HTTPException as e:
        # 重新抛出已有的 HTTP 异常
        raise e
    except Exception as e:
        # 捕获其他异常，并返回 500 错误
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")
