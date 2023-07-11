import logging
from typing import List

from PyQt5 import QtCore
import zmq

from ..data_chunk import DataChunk
from ..utils import deserialize, deserialize_image
from .comm_bus import CommBus

logger = logging.getLogger("pyorbslam")

class ThreadedZmqPoller(QtCore.QThread):
    
    def __init__(self, comm_bus: CommBus):
        super().__init__()

        # Save input parameters
        self.cbus = comm_bus

        # Create a poller
        self.running = True
        self.sub_poller = zmq.Poller()
        self.subs: List = []

        # Bind
        self.cbus.zeromqSub.connect(self.add_sub)
        self.cbus.closeApp.connect(self.shutdown)

    def add_sub(self, ip: str, port: int):

        # Create the sub
        _zmq_context = zmq.Context()
        _zmq_socket = _zmq_context.socket(zmq.SUB)
        _zmq_socket.setsockopt(zmq.CONFLATE, 1)
        _zmq_socket.connect(f"tcp://{ip}:{port}")
        _zmq_socket.subscribe(b"")

        # Add it to the poller
        self.sub_poller.register(_zmq_socket, zmq.POLLIN)

        # Store it
        self.subs.append(_zmq_socket)

    def shutdown(self):
        self.running = False

    def run(self):

        logger.debug(f"{self}: Polling thread running")

        while self.running:

            # Wait until we get data from any of the subscribers
            events = dict(self.sub_poller.poll(timeout=1000))
        
            # Empty if no events
            if len(events) == 0:
                # self.msleep(10)
                continue

            # Else, update values
            for s in events:  # socket

                # Update
                data_bytes = s.recv()
                data_chunk: DataChunk = deserialize(data_bytes)

                # Process the incoming data
                if data_chunk.vtype in ['line']:
                    self.cbus.dataUpdate.emit(data_chunk)
                elif data_chunk.vtype == 'image':
                    logger.debug(f"{self}: Received an image")
                    data_chunk.data = deserialize_image(data_chunk.data)
                    self.cbus.imageUpdate.emit(data_chunk.data)
