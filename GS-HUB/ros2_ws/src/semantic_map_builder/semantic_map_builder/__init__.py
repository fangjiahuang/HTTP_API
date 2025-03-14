import os

value = os.getenv("HUB_CONFIG_PATH")
if value:
    CONFIG_PATH = value
else:
    CONFIG_PATH = "/home/gs/workspace/projects/GS-HUB/configs/config.yaml"