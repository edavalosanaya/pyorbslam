import threading
import logging
from typing import List

import zmq

from ..data_chunk import DataChunk
from ..utils import deserialize
from .comm_bus import CommBus
from .display_3d import Display3D

logger = logging.getLogger("pyorbslam")

class ThreadedZmqPoller():
    
    def __init__(self, comm_bus: CommBus, window: Display3D):

        # Save input parameters
        self.cbus = comm_bus
        self.window = window

        # Create a poller
        self.running = True
        self.sub_poller = zmq.Poller()
        self.subs: List = []
        self.poll_thread = threading.Thread(target=self.poll_inputs)
        self.poll_thread.start()

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
        self.poll_thread.join()

    def poll_inputs(self):

        logger.debug(f"{self}: Polling thread running")

        while self.running:

            # Wait until we get data from any of the subscribers
            events = dict(self.sub_poller.poll(timeout=1000))
        
            # Empty if no events
            if len(events) == 0:
                continue

            # Else, update values
            for s in events:  # socket

                # Update
                data_bytes = s.recv()
                data: DataChunk = deserialize(data_bytes)

                # Process the incoming data
                # self.window.setData(data)
