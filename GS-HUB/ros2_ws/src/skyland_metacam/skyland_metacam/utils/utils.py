import uuid

def gen_uuid() -> str:
    return uuid.uuid4().hex