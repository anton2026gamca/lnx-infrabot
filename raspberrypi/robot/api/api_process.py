import multiprocessing.synchronize
import logging

from robot.api import api
from robot import vision



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    vision.register_detection_type(
        "goal_yellow",
        color=(255, 255, 0),
        label="Yellow Goal"
    )
    vision.register_detection_type(
        "goal_blue",
        color=(52, 125, 235),
        label="Blue Goal"
    )
    vision.register_detection_type(
        "ball",
        color=(255, 165, 0),
        label="Ball"
    )
    vision.register_detection_type(
        "ball_possession_area",
        color=(0, 255, 0),
        label="Possession Area",
        thickness=3
    )


    api.start(stop_event=stop_event)

