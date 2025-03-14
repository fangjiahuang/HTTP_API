import json

from typing import Dict

def serialize_response(data: Dict) -> str:
    return json.dumps(data, ensure_ascii=False)

def unserialize_response(data_string: str) -> Dict:
    try:
        return json.loads(data_string)
    except json.JSONDecodeError as e:
        print(f"Failed to decode JSON: {e}")
        return {}