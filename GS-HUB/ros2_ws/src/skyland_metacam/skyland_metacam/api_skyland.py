# api_2.py

from fastapi import FastAPI
from skyland_metacam.service.skyland_api import router as skyland_router

app = FastAPI()

# 将 `skyland_api.py` 中的路由添加到主应用
app.include_router(skyland_router)

if __name__ == "__main__":
    import uvicorn
    # 使用 Uvicorn 启动 FastAPI 应用
    uvicorn.run(app, host="0.0.0.0", port=5000)