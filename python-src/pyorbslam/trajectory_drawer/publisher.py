import logging
import time
import requests
import threading
import collections
from typing import Dict

import zmq

from .data_chunk import DataChunk
from .utils import get_ip_address, serialize, serialize_image

logger = logging.getLogger("pyorbslam")

class Publisher:

    def __init__(self, td_app_url: str):

        # Save input parameters
        self.td_app_url = td_app_url

        # Create ZeroMQ
        self._client_host = get_ip_address()
        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_port = self._zmq_socket.bind_to_random_port(f"tcp://{self._client_host}")
        time.sleep(0.5)

        # Inform server to subscribe to my 3D visuals
        response = requests.post(f"{td_app_url}/config/zeromq", json={'ip': self._client_host, 'port': self._zmq_port})
       
        # Reportin success/failure
        if response.status_code == requests.status_codes.codes.ok:
            logger.debug(f"{self}: Successful creating SUB/PUB connection!")
            time.sleep(1)
        else:
            logger.error(f"{self}: Failed to create SUB/PUB connection!")

    def send(self, data_chunk: DataChunk):

        if data_chunk.vtype == 'image':
            data_chunk.data = serialize_image(data_chunk.data)
        # elif data_chunk.vtype == 'point cloud':
        #     data_chunk.data = serialize_pc(data_chunk.data)

        # Send the data
        self._zmq_socket.send(serialize(data_chunk))
