'''
    建立qdrant向量数据库
'''

import os
import sys
import uuid
import time
import requests
import numpy as np
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, PointVectors, PointIdsList
from qdrant_client.models import Distance, VectorParams, SearchParams
from qdrant_client.models import Filter, FieldCondition, MatchValue

from typing import List, Dict, Any

class QdrantManager:
    """ 
    仅负责对 Qdrant 数据库的增删改查操作，不包含任何业务逻辑。
    """
    def __init__(self, client: QdrantClient):
        self.client = client
        self.collection_name = None

    def set_collection_name(self, collection_name: str):
        """设置当前操作的 collection。"""
        self.collection_name = collection_name
    
    def get_colletion_info(self):
        return self.client.get_collection(collection_name=self.collection_name)

    def create_collection(self, collection_name: str, dimension: int) -> bool:
        try:
            # 如果存在同名 collection，则删除
            if self.client.collection_exists(collection_name):
                is_success = self.client.delete_collection(collection_name=collection_name)
                assert is_success, "delete exist collection error"

            # 新建 collection
            is_success = self.client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=dimension, distance=Distance.COSINE),
            )
            assert is_success, "create collection error"

            return True
        except Exception as e:
            print("Err in create_collection:", str(e))
            return False

    def upsert_points(self, 
        ids: List[int], 
        vectors: List[List[float]], 
        payloads: List[Dict[str, Any]]
    ) -> bool:
        """
        批量插入或更新向量。
        :param ids: 对应的点 ID 列表
        :param vectors: 对应的向量列表
        :param payloads: 对应的 payload 列表
        """
        try:
            points = []
            print(type(ids[0], vectors[0], payloads[0]))
            for i in range(len(vectors)):
                points.append(
                    PointStruct(
                        id=ids[i],
                        vector=vectors[i],
                        payload=payloads[i],
                    )
                )
            operation_info = self.client.upsert(
                collection_name=self.collection_name,
                wait=True,
                points=points,
            )
            print(f"[QdrantManager] Upserted {len(points)} points.")
            print(operation_info)
            assert operation_info, "insert point error"
            return True
        except Exception as e:
            print("Err in upsert_points:", str(e))
            return False

    def upsert_single_point(self, 
        id: int, 
        vector: List[float], 
        payload: Dict[str, Any]
    ) -> bool:
        """插入或更新单条数据。"""
        try:
            operation_info = self.client.upsert(
                collection_name=self.collection_name,
                wait=True,
                points=[
                    PointStruct(id=id, vector=vector, payload=payload),
                ],
            )
            print(f"[QdrantManager] Upserted 1 document (ID: {id}).")
            assert operation_info, "insert point error"
            return True
        except Exception as e:
            print("Err in upsert_single_point:", str(e))
            return False

    def delete_points(self, ids: List[int]) -> bool:
        try:
            res = self.client.delete(
                collection_name=f"{self.collection_name}",
                points_selector=PointIdsList(
                    points=ids,
                ),
            )
            assert res, "delete point error"
            return True
        except Exception as e:
            print("Err in delete_points:", str(e))
            return False

    # 查询
    def search_points(self, vector: List[float], limit: int = 5):
        """搜索向量相似度最高的点。"""
        try:
            search_result = self.client.query_points(
                collection_name=self.collection_name,
                query=vector,
                with_payload=True,
                limit=limit
            ).points
            return search_result
        except Exception as e:
            print("Err in search_points:", str(e))
            return []

    def get_point_by_id(self, point_id: int):
        """
        在 qdrant_client 里通常是 query_points,
        但 query_points 需要 vector。
        如果想根据 ID 查询，你可以使用 client.retrieve 方法 
        或者使用 filter 进行查询。
        """
        try:
            result = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id],
                with_payload=True,
                with_vectors=True
            )
            return result
        except Exception as e:
            print("Err in get_point_by_id:", str(e))
            return None

    def update_point(self, 
        point_id: int, 
        vector: List[float], 
        payload: Dict[str, Any]
    ) -> bool:
        """更新指定 ID 的向量和/或 payload"""
        try:
            # 更新 payload
            set_payload_res = self.client.set_payload(
                collection_name=self.collection_name,
                payload=payload,
                points=[point_id]
            )
            # 更新向量
            update_vectors_res = self.client.update_vectors(
                collection_name=self.collection_name,
                points=[
                    PointVectors(
                        id=point_id,
                        vector=vector
                    )
                ]
            )
            assert set_payload_res and update_vectors_res, "update_point error"
            return True
        except Exception as e:
            print("Err in update_point:", str(e))
            return False

    def scroll_collection(self, limit=10):
        """分批次滚动查询，可以实现类似分页的功能。"""
        try:
            res = self.client.scroll(
                collection_name=self.collection_name,
                limit=limit,
                with_payload=True,
                with_vectors=True,
            )
            return res
        except Exception as e:
            print("Err in scroll_collection:", str(e))
            return None

if __name__ == '__main__':
    ...
   