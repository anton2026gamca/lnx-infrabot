import time
import logging
import multiprocessing.synchronize

from robot import calibration, utils
from robot.hardware import line_sensors, teensy
from robot.multiprocessing import shared_data
from robot.profiling import profile_function

from robot.hardware.teensy import TeensyCommunicator
from robot.config import *



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    messages_received = 0
    messages_sent = 0
    corrupted_messages = 0
    last_log_time = time.perf_counter()

    compass_offset: dict[str, int] = {
        "heading": 0,
        "pitch": 0,
        "roll": 0
    }

    attempts = 1
    attempt_start_time = time.perf_counter()

    with TeensyCommunicator(port=TEENSY_PORT, baud=TEENSY_BAUD, timeout=TEENSY_TIMEOUT) as communicator:
        while True:
            try:
                start_time = time.perf_counter()

                data = None

                new_messages = communicator.read_messages()

                if teensy.SENSOR_DATA_MESSAGE_TYPE in new_messages:
                    try:
                        data = teensy.parse_sensor_data_binary(new_messages[teensy.SENSOR_DATA_MESSAGE_TYPE])
                        if data.compass.heading != 999:
                            data.compass.heading = int(utils.normalize_angle_deg(data.compass.heading + compass_offset["heading"]))
                            data.compass.pitch = int(utils.normalize_angle_deg(data.compass.pitch + compass_offset["pitch"]))
                            data.compass.roll = int(utils.normalize_angle_deg(data.compass.roll + compass_offset["roll"]))
                        if data.ir.angle != 999:
                            data.ir.angle = int(utils.normalize_angle_deg(data.ir.angle + IR_BALL_ANGLE_OFFSET_DEG))
                        shared_data.set_hardware_data(data)
                        line_sensors.update_line_detected(data)
                        calibration.update_line_calibration(data)
                        attempt_start_time = 0
                        messages_received += 1
                    except ValueError as e:
                        logger.warning(f"Corrupted message: {new_messages[teensy.SENSOR_DATA_MESSAGE_TYPE]} - {e}")
                        corrupted_messages += 1
                        data = None
                if teensy.RUNNING_STATE_MESSAGE_TYPE in new_messages:
                    try:
                        shared_data.set_running_state(teensy.parse_running_state_binary(new_messages[teensy.RUNNING_STATE_MESSAGE_TYPE]))
                        messages_received += 1
                    except ValueError as e:
                        logger.warning(f"Corrupted message: {new_messages[teensy.RUNNING_STATE_MESSAGE_TYPE]} - {e}")
                        corrupted_messages += 1

                motor_speeds = shared_data.get_motor_speeds()
                kicker_state = shared_data.get_kicker_state()
                communicator.send_motors_message(motor_speeds, kicker_state)
                messages_sent += 1

                if data is None and attempt_start_time != 0 and attempt_start_time + 0.5 < time.perf_counter():
                    logger.warning(f"No data received from Teensy for 0.5 seconds, retrying connection... (attempt {attempts})")
                    communicator.close()
                    time.sleep(0.1)
                    communicator.connect()
                    attempt_start_time = time.perf_counter()
                    attempts += 1

                if data is not None and shared_data.check_and_clear_compass_reset():
                    logger.info("Resetting compass position")
                    compass_offset["heading"] -= data.compass.heading
                    compass_offset["pitch"] -= data.compass.pitch
                    compass_offset["roll"] -= data.compass.roll

                if time.perf_counter() > last_log_time + 1:
                    logger.debug(f"Messages - Recieved: {messages_received}, Sent: {messages_sent}, Corrupted: {corrupted_messages}")
                    messages_received = 0
                    messages_sent = 0
                    corrupted_messages = 0
                    last_log_time = time.perf_counter()

                time_elapsed = time.perf_counter() - start_time
                if time_elapsed < COMMUNICATION_LOOP_PERIOD:
                    time.sleep(max(0.0, COMMUNICATION_LOOP_PERIOD - time_elapsed - 0.001))
            except Exception as e:
                logger.error(f"{e}", exc_info=True)
                time.sleep(0.05)

            if stop_event.is_set():
                return

