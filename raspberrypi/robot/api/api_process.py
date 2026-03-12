import multiprocessing.synchronize
import logging

from robot.api import api



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    api.start(stop_event=stop_event)

