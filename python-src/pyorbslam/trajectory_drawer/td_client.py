import logging

import requests

logger = logging.getLogger("pyorbslam")

class TDClient:
    
    def __init__(self, port: int = 9000, ip: str = "127.0.0.1"):

        # Save parameter
        self.port = port
        self.ip = ip

    def __str__(self):
        return "<TDClient>"

    def __repr__(self):
        return str(self)

    @property
    def url(self):
        return f"http://{self.ip}:{self.port}"

    def shutdown(self):
        # http://localhost:9000
        response = requests.get(f"{self.url}/shutdown", timeout=10)
        logger.debug(f"Response status: {response.status_code}")

        if response.status_code == requests.status_codes.codes.ok:
            logger.debug(f"{self}: Successful shutdown: {response.json()}")
