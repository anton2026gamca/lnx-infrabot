import cv2
import logging
import multiprocessing.synchronize
import numpy as np
import time

from robot import calibration, vision
from robot.multiprocessing import shared_data

from robot.vision import GoalColorCalibration, DetectedObject
from robot.config import *



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    frames_processed = 0
    last_debug_msg_time = time.time()
    last_frame_timestamp = None
    frame_skip_count = 0
    
    target_period = 1.0 / CAMERA_MAX_FPS
    last_process_time = time.time()

    while not stop_event.is_set():
        elapsed = time.time() - last_process_time
        if elapsed < target_period * 0.95:
            time.sleep(max(0.0, target_period - elapsed - 0.0005))
            continue
        
        last_process_time = time.time()

        frame = shared_data.get_camera_frame()
        if frame is None:
            time.sleep(0.001)
            continue
        
        if last_frame_timestamp is not None and frame.timestamp == last_frame_timestamp:
            frame_skip_count += 1
            if frame_skip_count > 2:
                time.sleep(0.001)
                continue
        else:
            frame_skip_count = 0
        
        last_frame_timestamp = frame.timestamp
        
        hsv_frame = cv2.resize(frame.frame, (DETECTION_FRAME_WIDTH, DETECTION_FRAME_HEIGHT))
        hsv_frame = cv2.cvtColor(hsv_frame, cv2.COLOR_RGB2HSV)

        goal_color = shared_data.get_goal_color()
        lower, upper = shared_data.get_goal_calibration(goal_color)
        color_range = GoalColorCalibration(
            yellow_lower=np.array(lower) if goal_color.lower() == 'yellow' else GoalColorCalibration().yellow_lower,
            yellow_upper=np.array(upper) if goal_color.lower() == 'yellow' else GoalColorCalibration().yellow_upper,
            blue_lower=np.array(lower) if goal_color.lower() == 'blue' else GoalColorCalibration().blue_lower,
            blue_upper=np.array(upper) if goal_color.lower() == 'blue' else GoalColorCalibration().blue_upper
        )
        focal_length = shared_data.get_goal_focal_length()
        [result, goal_detections] = vision.detect_goal_alignment_with_rect(
            hsv_frame,
            goal_color,
            color_range,
            focal_length_pixels=focal_length,
            real_goal_height_mm=GOAL_HEIGHT_MM
        )
        shared_data.set_goal_detection_result(result)
        calibration.update_goal_distance_calibration(result)

        ball_lower, ball_upper = shared_data.get_ball_calibration()
        ball_detections, _ = vision.detect_ball(hsv_frame, np.array(ball_lower), np.array(ball_upper))

        ball_data = vision.calculate_ball_data(ball_detections, DETECTION_FRAME_WIDTH, CAMERA_FOV_DEG, shared_data.get_camera_ball_calibration_constant())
        shared_data.set_camera_ball_data(ball_data)

        ball_center_x = ball_detections[0].x + ball_detections[0].width / 2 if ball_detections else None
        ball_center_y = ball_detections[0].y + ball_detections[0].height / 2 if ball_detections else None

        ball_possessed, possession_area = vision.detect_ball_possession(
            ball_center_x, ball_center_y,
            DETECTION_FRAME_WIDTH, DETECTION_FRAME_HEIGHT,
            AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT,
            AUTO_BALL_POSSESSION_AREA_HEIGHT_PERCENT,
        )
        shared_data.set_camera_ball_possession(ball_possessed)

        all_detections = goal_detections + ball_detections
        possession_area_color = (0, 255, 0) if ball_possessed else (0, 0, 255)
        possession_detection = DetectedObject(
            object_type="ball_possession_area",
            x=possession_area.x,
            y=possession_area.y,
            width=possession_area.width,
            height=possession_area.height,
            color=possession_area_color
        )
        all_detections.append(possession_detection)

        for det in all_detections:
            det.x = int(det.x / DETECTION_FRAME_SIZE_SCALE)
            det.y = int(det.y / DETECTION_FRAME_SIZE_SCALE)
            det.width = int(det.width / DETECTION_FRAME_SIZE_SCALE)
            det.height = int(det.height / DETECTION_FRAME_SIZE_SCALE)

        if ball_possessed and ball_detections:
            for ball_det in ball_detections:
                ball_det.color = (0, 255, 0)

        shared_data.set_detected_objects(all_detections)

        frames_processed += 1
        
        if time.time() > last_debug_msg_time + 1:
            logger.debug(f"Camera Processing FPS: {frames_processed}")
            frames_processed = 0
            last_debug_msg_time += 1


