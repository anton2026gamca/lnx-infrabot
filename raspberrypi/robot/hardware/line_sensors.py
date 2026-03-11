import robot.multiprocessing.shared_data as shared_data
from robot.config import *
from robot.hardware.teensy import ParsedTeensyData



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

