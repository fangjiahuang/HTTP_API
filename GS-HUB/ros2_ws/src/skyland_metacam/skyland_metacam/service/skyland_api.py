from fastapi import APIRouter, HTTPException, Query
# 导入 FastAPI 的相关组件：
# - `APIRouter`：用于定义路由（API的路径）。
# - `HTTPException`：用于抛出HTTP异常（如404、500错误）。
# - `Query`：用于从URL中提取查询参数。

from pydantic import BaseModel
# 导入 Pydantic 的 `BaseModel`，用于定义请求数据的结构和验证。

from typing import List, Optional
# 导入 `List` 和 `Optional`，用于类型注解：
# - `List`：表示列表类型。
# - `Optional`：表示某个字段可以为 `None`。

import requests
# 导入 `requests` 库，用于发送HTTP请求（虽然当前代码未直接使用）。

# 假设 SkylandCameraAPI 已经在其他地方定义好并可以直接使用
from skyland_metacam.service.skyland_service import SkylandCameraAPI
# 从 `skyland_service` 模块中导入 `SkylandCameraAPI` 类，用于与感知塔通信。



# 定义一个 FastAPI 的路由实例，所有以 `/v1/skyland` 开头的路径都会由这个路由器处理。
router = APIRouter(prefix="/v1/skyland", tags=["skyland"])

# 初始化 SkylandCameraAPI 实例
skyland_api = SkylandCameraAPI(metacam_ip="127.0.0.1")  
# 创建一个 `SkylandCameraAPI` 对象，默认与 IP 地址为 `127.0.0.1` 的感知塔通信。
# 如果需要连接其他IP地址，可以修改这里的 `metacam_ip` 参数。



### ​**定义请求模型**

class CaptureModeRequest(BaseModel):
    """
    请求模型，用于设置捕获模式。
    """
    image_quality: int = 75
    # 定义一个字段 `image_quality`，表示图像质量，默认值为75。
    resolution_division: int = 2
    # 定义一个字段 `resolution_division`，表示分辨率分割，默认值为2。
    in_rosbag: bool = True
    # 定义一个字段 `in_rosbag`，表示是否将数据保存到 ROS bag 文件中，默认值为 True。

class SaveModeRequest(BaseModel):
    """
    请求模型，用于设置保存模式。
    """
    save_interval: int = 1
    # 定义一个字段 `save_interval`，表示保存间隔，默认值为1。
    distance_moved: float = 0.0
    # 定义一个字段 `distance_moved`，表示移动距离，默认值为0.0。
    angle_rotated: float = 0.0
    # 定义一个字段 `angle_rotated`，表示旋转角度，默认值为0.0。
    blur_score: float = 0.0
    # 定义一个字段 `blur_score`，表示模糊分数，默认值为0.0。

class NavConfigRequest(BaseModel):
    """
    请求模型，用于设置导航配置。
    """
    world_point: List[float]  # [x, y, z]
    # 定义一个字段 `world_point`，表示目标点的坐标，类型为浮点数列表（如 [1.0, 2.0, 3.0]）。
    mode: int = 0
    # 定义一个字段 `mode`，表示导航模式，默认值为0。
    parameters: List[float] = [1]
    # 定义一个字段 `parameters`，表示导航参数，默认值为 [1]。



### ​**定义路由和处理函数**

@router.post("/set_capture_mode")
async def set_capture_mode_endpoint(request: CaptureModeRequest):
    """
    设置相机的捕获模式。
    """
    try:
        # 调用 `SkylandCameraAPI` 的 `set_capture_mode` 方法，传入请求中的参数。
        response = skyland_api.set_capture_mode(
            image_quality=request.image_quality,
            resolution_division=request.resolution_division,
            in_rosbag=request.in_rosbag
        )
        # 如果返回值为 None，说明设置失败，抛出 HTTP 500 错误。
        if response is None:
            raise HTTPException(status_code=500, detail="Failed to set capture mode.")
        # 返回成功响应，包含返回的数据。
        return {"code": 0, "msg": "success", "data": response}
    except Exception as e:
        # 如果发生异常，抛出 HTTP 500 错误，并返回错误信息。
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")



@router.post("/set_save_mode")
async def set_save_mode_endpoint(request: SaveModeRequest):
    """
    设置相机的保存模式。
    """
    try:
        # 调用 `SkylandCameraAPI` 的 `set_save_mode` 方法，传入请求中的参数。
        response = skyland_api.set_save_mode(
            save_interval=request.save_interval,
            distance_moved=request.distance_moved,
            angle_rotated=request.angle_rotated,
            blur_score=request.blur_score
        )
        # 如果返回值为 None，说明设置失败，抛出 HTTP 500 错误。
        if response is None:
            raise HTTPException(status_code=500, detail="Failed to set save mode.")
        # 返回成功响应，包含返回的数据。
        return {"code": 0, "msg": "success", "data": response}
    except Exception as e:
        # 如果发生异常，抛出 HTTP 500 错误，并返回错误信息。
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")



@router.get("/start_recording")
async def start_recording_endpoint():
    """
    开始录制。
    """
    try:
        # 调用 `SkylandCameraAPI` 的 `start_recording` 方法。
        response = skyland_api.start_recording()
        # 如果返回值为 None，说明开始录制失败，抛出 HTTP 500 错误。
        if response is None:
            raise HTTPException(status_code=500, detail="Failed to start recording.")
        # 返回成功响应，包含返回的数据。
        return {"code": 0, "msg": "success", "data": response}
    except Exception as e:
        # 如果发生异常，抛出 HTTP 500 错误，并返回错误信息。
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")



@router.post("/set_nav_config")
async def set_nav_config_endpoint(request: NavConfigRequest):
    """
    设置导航配置。
    """
    try:
        # 调用 `SkylandCameraAPI` 的 `set_nav_config` 方法，传入请求中的参数。
        response = skyland_api.set_nav_config(
            world_point=request.world_point,
            mode=request.mode,
            parameters=request.parameters
        )
        # 如果返回值为 None，说明设置失败，抛出 HTTP 500 错误。
        if response is None:
            raise HTTPException(status_code=500, detail="Failed to set navigation config.")
        # 返回成功响应，包含返回的数据。
        return {"code": 0, "msg": "success", "data": response}
    except Exception as e:
        # 如果发生异常，抛出 HTTP 500 错误，并返回错误信息。
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")



@router.get("/start_navigation")
async def start_navigation_endpoint():
    """
    开始导航。
    """
    try:
        # 调用 `SkylandCameraAPI` 的 `start_navigation` 方法。
        response = skyland_api.start_navigation()
        # 如果返回值为 None，说明开始导航失败，抛出 HTTP 500 错误。
        if response is None:
            raise HTTPException(status_code=500, detail="Failed to start navigation.")
        # 返回成功响应，包含返回的数据。
        return {"code": 0, "msg": "success", "data": response}
    except Exception as e:
        # 如果发生异常，抛出 HTTP 500 错误，并返回错误信息。
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")



@router.get("/stop_navigation")
async def stop_navigation_endpoint():
    """
    停止导航。
    """
    try:
        # 调用 `SkylandCameraAPI` 的 `stop_navigation` 方法。
        response = skyland_api.stop_navigation()
        # 如果返回值为 None，说明停止导航失败，抛出 HTTP 500 错误。
        if response is None:
            raise HTTPException(status_code=500, detail="Failed to stop navigation.")
        # 返回成功响应，包含返回的数据。
        return {"code": 0, "msg": "success", "data": response}
    except Exception as e:
        # 如果发生异常，抛出 HTTP 500 错误，并返回错误信息。
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")



@router.post("/set_domain_id")
async def set_domain_id_endpoint(domain_id: int = Query(42)):
    """
    设置域 ID。
    """
    try:
        # 调用 `SkylandCameraAPI` 的 `set_domain_id` 方法，传入域 ID。
        skyland_api.set_domain_id(domain_id=domain_id)
        # 返回成功响应，包含设置的域 ID。
        return {"code": 0, "msg": "success", "data": {"domain_id": domain_id}}
    except Exception as e:
        # 如果发生异常，抛出 HTTP 500 错误，并返回错误信息。
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")
