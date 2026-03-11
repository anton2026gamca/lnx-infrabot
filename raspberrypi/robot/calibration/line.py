import robot.multiprocessing.shared_data as shared_data
import robot.utils as utils
from robot.calibration.data_manager import save_calibration_data
from robot.config import *
from robot.hardware.teensy import ParsedTeensyData



logger = utils.get_logger("Line Calibration")


def start_line_calibration(phase: int = 1) -> None:
    with shared_data.line_calibration_lock:
        if phase == 1:
            shared_data.line_calibration_phase.value = 1
            for i in range(LINE_SENSOR_COUNT):
                shared_data.line_calibration_min[i] = float('inf')
                shared_data.line_calibration_max[i] = float('-inf')
                shared_data.line_calibration_phase1_min[i] = float('inf')
                shared_data.line_calibration_phase1_max[i] = float('-inf')
                shared_data.line_calibration_phase2_min[i] = float('inf')
                shared_data.line_calibration_phase2_max[i] = float('-inf')
        elif phase == 2:
            shared_data.line_calibration_phase.value = 2
            for i in range(LINE_SENSOR_COUNT):
                shared_data.line_calibration_phase1_min[i] = shared_data.line_calibration_min[i]
                shared_data.line_calibration_phase1_max[i] = shared_data.line_calibration_max[i]
                shared_data.line_calibration_min[i] = float('inf')
                shared_data.line_calibration_max[i] = float('-inf')
    logger.info(f"Line sensor calibration phase {phase} started")

def stop_line_calibration(cancel: bool = False) -> tuple[list[list[int]], list[float], list[float], int]:
    with shared_data.line_calibration_lock:
        phase = shared_data.line_calibration_phase.value
        
        thresholds = []
        min_values = shared_data.line_calibration_min[:]
        max_values = shared_data.line_calibration_max[:]
        
        if phase == 1:
            for i in range(LINE_SENSOR_COUNT):
                if min_values[i] != float('inf') and max_values[i] != float('-inf'):
                    thresholds.append([int(min_values[i]), int(max_values[i])])
                else:
                    existing_min = shared_data.line_detection_thresholds[i * 2]
                    existing_max = shared_data.line_detection_thresholds[i * 2 + 1]
                    thresholds.append([existing_min, existing_max])
                shared_data.line_calibration_phase1_min[i] = shared_data.line_calibration_min[i]
                shared_data.line_calibration_phase1_max[i] = shared_data.line_calibration_max[i]

            shared_data.line_calibration_phase.value = 0
        elif phase == 2:
            for i in range(LINE_SENSOR_COUNT):
                shared_data.line_calibration_phase2_min[i] = shared_data.line_calibration_min[i]
                shared_data.line_calibration_phase2_max[i] = shared_data.line_calibration_max[i]
            
            for i in range(LINE_SENSOR_COUNT):
                field_min = shared_data.line_calibration_phase1_min[i]
                field_max = shared_data.line_calibration_phase1_max[i]
                line_min = shared_data.line_calibration_phase2_min[i]
                line_max = shared_data.line_calibration_phase2_max[i]
                
                if field_min != float('inf') and field_max != float('-inf'):
                    if line_min != float('inf') and line_max != float('-inf'):
                        field_center = (field_min + field_max) / 2
                        line_center = (line_min + line_max) / 2

                        field_range = field_max - field_min
                        margin = max(10, field_range * 0.1)

                        new_min = field_min
                        new_max = field_max

                        if line_center - field_center > margin:
                            new_min = 0
                            new_max = int(field_max + line_max) // 2
                        elif field_center - line_center > margin:
                            new_min = int(field_min + line_min) // 2
                            new_max = 1000
                        
                        thresholds.append([int(new_min), int(new_max)])
                    else:
                        thresholds.append([int(field_min), int(field_max)])
                else:
                    existing_min = shared_data.line_detection_thresholds[i * 2]
                    existing_max = shared_data.line_detection_thresholds[i * 2 + 1]
                    thresholds.append([existing_min, existing_max])
            
            shared_data.line_calibration_phase.value = 0

    if cancel:
        shared_data.line_calibration_phase.value = 0
        logger.info(f"Line sensor calibration phase {phase} cancelled")
    elif phase > 0:
        for i in range(LINE_SENSOR_COUNT):
            shared_data.line_detection_thresholds[i * 2] = thresholds[i][0]
            shared_data.line_detection_thresholds[i * 2 + 1] = thresholds[i][1]
        save_calibration_data()
        logger.info(f"Line sensor calibration phase {phase} completed and applied. Thresholds: {thresholds}")
    
    
    return thresholds, min_values, max_values, phase

def get_line_calibration_status() -> dict:
    with shared_data.line_calibration_lock:
        flat = shared_data.line_detection_thresholds[:]
        current_thresholds = [[flat[i*2], flat[i*2+1]] for i in range(LINE_SENSOR_COUNT)]
        phase = shared_data.line_calibration_phase.value
        
        def sanitize_values(values):
            if values is None:
                return None
            return [None if (v == float('inf') or v == float('-inf')) else v for v in values]
        
        calibration_min = shared_data.line_calibration_min[:] if phase > 0 else None
        calibration_max = shared_data.line_calibration_max[:] if phase > 0 else None

        phase1_complete = shared_data.line_calibration_phase1_min[:][0] != float('inf')
        phase2_complete = shared_data.line_calibration_phase2_min[:][0] != float('inf')
        
        return {
            'active': phase > 0,
            'phase': phase,
            'current_thresholds': current_thresholds,
            'calibration_min': sanitize_values(calibration_min),
            'calibration_max': sanitize_values(calibration_max),
            'phase1_complete': phase1_complete,
            'phase1_min': sanitize_values(shared_data.line_calibration_phase1_min[:]) if phase1_complete else None,
            'phase1_max': sanitize_values(shared_data.line_calibration_phase1_max[:]) if phase1_complete else None,
            'phase2_complete': phase2_complete,
            'phase2_min': sanitize_values(shared_data.line_calibration_phase2_min[:]) if phase2_complete else None,
            'phase2_max': sanitize_values(shared_data.line_calibration_phase2_max[:]) if phase2_complete else None,
        }

def update_line_calibration(data: ParsedTeensyData):
    with shared_data.line_calibration_lock:
        if shared_data.line_calibration_phase.value > 0:
            for i, value in enumerate(data.line):
                if i < LINE_SENSOR_COUNT:
                    shared_data.line_calibration_min[i] = min(shared_data.line_calibration_min[i], value)
                    shared_data.line_calibration_max[i] = max(shared_data.line_calibration_max[i], value)

def set_line_detection_thresholds(thresholds: list[list[int]]) -> None:
    shared_data.set_line_detection_thresholds(thresholds)
    save_calibration_data()
    logger.info(f"Line sensor calibration thresholds manually set to: {thresholds}")

