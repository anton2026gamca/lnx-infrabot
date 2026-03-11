import multiprocessing.synchronize
import logging
try:
    import web_server.api as api
except ImportError:
    api = None



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    if api is None:
        logger.warning("API module not found, API will not be started.")
        return
    api.start(stop_event=stop_event)

