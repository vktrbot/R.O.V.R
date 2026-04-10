import logging
from config_resolver import resolve_config
from constants import *
from connector import ServerWrapper
import uvicorn

if __name__=="__main__":
    logger = logging.getLogger("service")
    settings = resolve_config(SETTINGS_NAME)
    
    port = settings["port"]
    location = settings["cfg_location"]
    cfg = resolve_config(location)
    service = ServerWrapper(cfg)
    uvicorn.run(service.app, host="0.0.0.0", port=port)