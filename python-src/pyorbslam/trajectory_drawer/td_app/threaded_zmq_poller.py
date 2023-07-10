import zmq

from .comm_bus import CommBus

class ThreadedZmqPoller():
    
    def __init__(self, comm_bus: CommBus):

        # Save input parameters
        self.cbus = comm_bus

        # Create a poller
        self.sub_poller = zmq.Poller()
