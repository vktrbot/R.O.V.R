import os
from benedict import benedict

def resolve_config(config_name):
    path = os.path.join(os.path.dirname(__file__), config_name)
    config = benedict.from_yaml(path)
    return config