import time
import logging
import multiprocessing.synchronize
import robot.hardware.teensy as teensy
import robot.multiprocessing.shared_data as shared_data
import robot.utils as utils
import robot.calibration as calibration
import robot.hardware.line_sensors as line_sensors
from robot.hardware.teensy import TeensyCommunicator
from robot.config import *



def run(stop_event: multiprocessing.synchronize.Event, logger: logging.Logger):
    messages_received = 0
    messages_sent = 0
    last_log_time = time.time()

    compass_offset: dict[str, int] = {
        "heading": 0,
        "pitch": 0,
        "roll": 0
    }

    with TeensyCommunicator(port=TEENSY_PORT, baud=TEENSY_BAUD, timeout=TEENSY_TIMEOUT) as communicator:
        while not stop_event.is_set():
            start_time = time.time()

            line = communicator.read_line()
            data = None

            if line:
                messages_received += 1

                line_type = teensy.get_line_type(line)

                if line_type == teensy.LineType.SENSOR_DATA:
                    data = teensy.parse_sensor_data_line(line)
                elif line_type == teensy.LineType.RUNNING_STATE:
                    state = teensy.parse_running_state_line(line)
                    shared_data.set_running_state(state)
                
                communicator.send_motors_message(shared_data.get_motor_speeds(), shared_data.get_kicker_state())
                messages_sent += 1

            if data:
                if data.compass.heading != 999:
                    data.compass.heading -= compass_offset["heading"]
                    data.compass.pitch -= compass_offset["pitch"]
                    data.compass.roll -= compass_offset["roll"]
                    data.compass.heading = int(utils.normalize_angle_deg(data.compass.heading))
                
                if data.ir.angle != 999:
                    data.ir.angle = int(utils.normalize_angle_deg(data.ir.angle + IR_BALL_ANGLE_OFFSET_DEG))
                
                calibration.update_line_calibration(data)
                line_sensors.update_line_detected(data)
                shared_data.set_hardware_data(data)


            if shared_data.check_and_clear_compass_reset():
                if not data:
                    data = shared_data.get_hardware_data()
                if data:
                    logger.info("Resetting compass position")
                    compass_offset["heading"] += data.compass.heading
                    compass_offset["pitch"] += data.compass.pitch
                    compass_offset["roll"] += data.compass.roll

            if time.time() > last_log_time + 1:
                logger.debug(f"Messages received per second: {messages_received}")
                logger.debug(f"Messages sent per second: {messages_sent}")
                messages_received = 0
                messages_sent = 0
                last_log_time = time.time()

            time_elapsed = time.time() - start_time
            if time_elapsed < COMMUNICATION_LOOP_PERIOD:
                time.sleep(max(0.0, COMMUNICATION_LOOP_PERIOD - time_elapsed - 0.001))

