from robot.multiprocessing import shared_data

from robot.hardware.teensy import ParsedTeensyData
from robot.config import *



def get_line_detected() -> list[bool]:
    with shared_data.line_detected_lock:
        return shared_data.line_detected[:]

def update_line_detected(data: ParsedTeensyData):
    with shared_data.line_calibration_lock:
        for i, value in enumerate(data.line):
            if i >= LINE_SENSOR_COUNT:
                break
            threshold_min = shared_data.line_detection_thresholds[i * 2]
            threshold_max = shared_data.line_detection_thresholds[i * 2 + 1]
            shared_data.line_detected[i] = value < threshold_min or value > threshold_max

