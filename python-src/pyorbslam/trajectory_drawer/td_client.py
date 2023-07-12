import logging
from typing import Literal, Any, Dict

import numpy as np
import requests

from .publisher import Publisher
from .data_chunk import DataChunk

logger = logging.getLogger("pyorbslam")

class TDClient:
    
    visuals: Dict[str, Any] # Dict[str, Item]
    
    def __init__(self, port: int = 9000, ip: str = "127.0.0.1"):

        # Save parameter
        self.port = port
        self.ip = ip
        
        # Containers
        self.visuals = {}

        # Create publisher
        self.publisher = Publisher(self.url)

    def __str__(self):
        return "<TDClient>"

    def __repr__(self):
        return str(self)

    @property
    def url(self):
        return f"http://{self.ip}:{self.port}"

    def create_visual(self, name: str, vtype: Literal['line', 'mesh'], data: Any):

        # Send information to create visualization via HTTP
        response = requests.post(f"{self.url}/visuals/create", json={'name': name, 'vtype': vtype})

        if response.status_code == requests.status_codes.codes.ok:
            # Then send the actual data via ZeroMQ
            self.publisher.send(DataChunk(name, vtype, data))

            # Record the creation
            self.visuals[name] = data
        
            # Then send the updated visual (and because sometimes the first zmq message fails)
            self.update_visual(name, vtype, data)

        else:
            logger.debug(f"{self}: Failed to create visual: {name}")

    def update_visual(self, name: str, vtype: Literal['line', 'mesh'], data: Any):

        if name not in self.visuals:
            logger.warning(f"{self}: Failed to update visual (not created yet): {name}")
            return None

        self.publisher.send(DataChunk(name, vtype, data))
        # logger.debug(f"{self}: Sent visual ({name}) via ZeroMQ")
             

    def delete_visual(self, name: str):
        # Send information to create visualization via HTTP
        response = requests.post(f"{self.url}/visuals/delete", json={'name': name})

        if response.status_code != requests.status_codes.codes.ok:
            logger.error(f"{self}: Failed to delete visual: {name}")

    def send_image(self, image: np.ndarray):
        self.publisher.send(DataChunk('image', 'image', image))

    def shutdown(self):
        response = requests.get(f"{self.url}/shutdown", timeout=0.1)

        if response.status_code == requests.status_codes.codes.ok:
            logger.debug(f"{self}: Successful shutdown")
