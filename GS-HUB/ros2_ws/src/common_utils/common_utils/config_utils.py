import os

import yaml

class ConfigManager:
    def __init__(self, config_dir=None):
        # 默认加载 `configs` 文件夹下的配置文件
        self.config_dir = config_dir or os.path.join(os.getcwd(), "configs")
        self.config_cache = {}

    def load_config(self, filename):
        """加载指定配置文件"""
        if filename in self.config_cache:
            return self.config_cache[filename]
        
        config_path = os.path.join(self.config_dir, filename)
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file '{filename}' not found in '{self.config_dir}'")

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
            self.config_cache[filename] = config
            return config

    def get_config(self, filename):
        """获取配置文件中的特定项"""
        config = self.load_config(filename)
        return config