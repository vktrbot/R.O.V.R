import uuid

def generate_str(length: int):
    return str(uuid.uuid4()).replace('-', '')[:length]

def convert_message(type: str, content: str):
    return {"type": type, "content": content}