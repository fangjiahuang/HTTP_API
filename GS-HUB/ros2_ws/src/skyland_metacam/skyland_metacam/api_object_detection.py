from fastapi import FastAPI
from skyland_metacam.service.object_detection_api import router as object_detection_router

# 创建 FastAPI 应用
app = FastAPI()

# 添加 object_detection_api 的路由
app.include_router(object_detection_router)

# 启动服务时运行此文件
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="debug")
