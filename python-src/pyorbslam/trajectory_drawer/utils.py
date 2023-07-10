import logging
import pickle

import netifaces as ni
import blosc

logger = logging.getLogger("pyorbslam")

def get_ip_address() -> str:

    # Get gateway of the network
    gws = ni.gateways()
    try:
        default_gw_name = gws["default"][ni.AF_INET][1]
        # Get the ip in the default gateway
        ip = ni.ifaddresses(default_gw_name)[ni.AF_INET][0]["addr"]
    except KeyError:
        logger.warning("pyorbslam: Couldn't find connected network, using 127.0.0.1")
        ip = "127.0.0.1"

    return ip

def serialize(data):
    return blosc.compress(pickle.dumps(data, protocol=pickle.HIGHEST_PROTOCOL))

def deserialize(data_bytes):
    return pickle.loads(blosc.decompress(data_bytes))
