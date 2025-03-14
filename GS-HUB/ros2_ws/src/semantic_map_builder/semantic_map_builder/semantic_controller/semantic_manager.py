
from typing import Dict,List, Any

from qdrant_client import QdrantClient
from common_utils import ConfigManager, QdrantManager

class SemanticManger():
    def __init__(self, db_client: QdrantClient):
        self.qdrant_manager = QdrantManager(db_client)
        self.collection_name = None

    def init_vecdb(self, collection_name: str, dimension: int) -> bool:
        """
        初始化向量数据库：如果有旧的就删除，新建一个新的。
        这里是一个业务操作的入口。
        """
        created = self.qdrant_manager.create_collection(collection_name, dimension)
        if created:
            # 设置当前正在使用的 collection
            self.collection_name = collection_name
            self.qdrant_manager.set_collection_name(collection_name)
            print(f"[SemanticManager] Collection '{collection_name}' created and set.")
            return True
        else:
            print(f"[SemanticManager] Failed to create collection '{collection_name}'.")
            return False
    
    def load_vecdb(self, collection_name: str):
        self.qdrant_manager.set_collection_name(collection_name)
        return 

    def map_builder(self, 
        ids: List[int],
        vectors: List[List[float]],
        payloads: List[Dict[str, Any]]
    ) -> bool:
        
        if not self.collection_name:
            print("[SemanticManager] No collection set. Please init_vecdb first.")
            return False

        # 业务层可以做一些验证或逻辑，比如检查长度是否一致
        if not (len(ids) == len(vectors) == len(payloads)):
            print("[SemanticManager] Input length mismatch.")
            return False
        
        return self.qdrant_manager.upsert_points(ids, vectors, payloads)

    def get_vecdb_counts(self):
        """
        只返回向量数据库中的数据数量
        """
        res = self.get_vecdb_status()
        return res['result'].get("points_count")

    def get_vecdb_status(self):
        """
        获取向量数据库信息
        """
        return self.qdrant_manager.get_colletion_info()