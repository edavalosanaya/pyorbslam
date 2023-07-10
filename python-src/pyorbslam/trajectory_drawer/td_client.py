import logging
import time
from typing import Literal, Any, Dict

import zmq
import requests

from .data_chunk import DataChunk
from .utils import get_ip_address, serialize

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
        self._client_host = get_ip_address()
        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_port = self._zmq_socket.bind_to_random_port(f"tcp://{self._client_host}")
        time.sleep(0.25)

        # Inform server to subscribe to my 3D visuals
        # response = requests.post(f"{self.url}/config/zeromq", json={'ip': self._client_host, 'port': self._zmq_port})
        
        # if response.status_code != requests.status_codes.codes.ok:
        #     logger.error(f"{self}: Failed to create SUB/PUB connection!")

    def __str__(self):
        return "<TDClient>"

    def __repr__(self):
        return str(self)

    @property
    def url(self):
        return f"http://{self.ip}:{self.port}"

    def create_visual(self, name: str, vtype: Literal['line'], data: Any):

        # Send information to create visualization via HTTP
        response = requests.post(f"{self.url}/visuals/create", json={'name': name, 'vtype': vtype})

        if response.status_code == requests.status_codes.codes.ok:
            # Then send the actual data via ZeroMQ
            self._zmq_socket.send(serialize(DataChunk(name, vtype, data)))

            # Record the creation
            self.visuals[name] = data

        else:
            logger.debug(f"{self}: Failed to create visual: {name}")

    def update_visual(self, name: str, vtype: Literal['line'], data: Any):

        if name not in self.visuals:
            logger.warning(f"{self}: Failed to update visual (not created yet): {name}")
            return None

        self._zmq_socket.send(serialize(DataChunk(name, vtype, data)))
             

    def delete_visual(self, name: str):
        # Send information to create visualization via HTTP
        response = requests.post(f"{self.url}/visuals/delete", json={'name': name})

        if response.status_code != requests.status_codes.codes.ok:
            logger.error(f"{self}: Failed to delete visual: {name}")

    def shutdown(self):
        response = requests.get(f"{self.url}/shutdown", timeout=5)

        if response.status_code == requests.status_codes.codes.ok:
            logger.debug(f"{self}: Successful shutdown")
