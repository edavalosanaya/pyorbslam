import logging.config

LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": True,
    "formatters": {
        "standard": {
            "format": "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
            "datefmt": "%Y-%m-%d %H:%M:%S",
        },
    },
    "handlers": {
        "console": {
            "level": "DEBUG",
            "formatter": "standard",
            "class": "logging.StreamHandler",
            "stream": "ext://sys.stdout",  # Default is stderr
        },
    },
    "loggers": {
        "pyorbslam": {
            "handlers": ["console"],
            "level": "DEBUG",
            "propagate": True,
        },
    },
}


# Setup the logging configuration
def setup():

    # Setting up the configureation
    logging.config.dictConfig(LOGGING_CONFIG)
