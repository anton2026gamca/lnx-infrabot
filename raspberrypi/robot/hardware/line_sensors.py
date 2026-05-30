from __future__ import annotations

from robot.multiprocessing import shared_data
from robot.profiling import profile_function

from robot.hardware.teensy import ParsedTeensyData
from robot.config import *



@profile_function
def get_line_detected() -> list[bool]:
    with shared_data.line_detected_lock:
        return shared_data.line_detected[:]

@profile_function
def update_line_detected(data: ParsedTeensyData) -> None:
    detected = []
    with shared_data.line_calibration_lock:
        for i, value in enumerate(data.line):
            if i >= LINE_SENSOR_COUNT:
                break
            threshold_min = shared_data.line_detection_thresholds[i * 2]
            threshold_max = shared_data.line_detection_thresholds[i * 2 + 1]
            is_detected = not (threshold_min <= value <= threshold_max)
            detected.append(is_detected)
    with shared_data.line_detected_lock:
        for i in range(LINE_SENSOR_COUNT):
            shared_data.line_detected[i] = detected[i]
