$fn = 90;

use <bottom_part.scad>;
use <middle_part.scad>;
use <upper_part.scad>;
use <bottom_wall.scad>;
use <middle_wall.scad>;
use <ball_zone.scad>;
use <handle_bracket.scad>;
use <handle.scad>;
use <uppest_part.scad>;
use <ir_sensor.scad>;
use <camera_bracket.scad>;
use <line_sensor_shield.scad>;
use <motor_driver_bracket.scad>;


module camera_front_bracket() {
    translate([0, -52.3, 104.3])
        rotate([20, 0, 0])
            camera_bracket();
}

module camera_back_bracket() {
    translate([0, 34, 186.3])
        rotate([20, 0, 180])
            camera_bracket();
}

module robot(
    wheel_d = 60,
    wheel_h = 12.5,
    wheel_offset = 90,
    robot_d = 215,
    robot_h = 220,
    rpi_pos = [0, -30, 55],
    level_shifter_pos = [-35, 32, 57.4],
    show_bottom = true,
    show_middle = true,
    show_upper = true,
    show_top = true,
    show_ball_zone = true,
    show_bottom_wall = true,
    show_middle_wall = true,
    show_handle = true,
    show_handle_bracket = true,
    show_camera_front = true,
    show_camera_back = true,
    show_motor_driver_brackets = true,
    show_line_sensor_shield = true,
    show_other_components = true
) {
    union() {
        if (show_bottom)
            bottom_part(robot_d = robot_d, wheel_d = wheel_d, wheel_h = wheel_h, wheel_offset = wheel_offset);
        if (show_bottom_wall)
            bottom_wall();
        if (show_ball_zone)
            ball_zone(robot_d = robot_d, robot_h = robot_h, wheel_d = wheel_d, wheel_h = wheel_h, wheel_offset = wheel_offset);
        if (show_middle)
            middle_part(
                robot_d = robot_d,
                wheel_offset = wheel_offset,
                rpi_pos = rpi_pos,
                level_shifter_pos = level_shifter_pos,
                show_raspberry = show_other_components
            );
        if (show_middle_wall)
            middle_wall();
        if (show_upper)
            upper_part(robot_d = robot_d, wheel_offset = wheel_offset);
        if (show_top)
            uppest_part(show_ir_sensor = show_other_components);
        if (show_handle)
            handle();
        if (show_handle_bracket)
            handle_bracket();
        if (show_camera_front)
            camera_front_bracket();
        if (show_camera_back)
            camera_back_bracket();
        if (show_motor_driver_brackets)
            motor_driver_brackets();
        if (show_line_sensor_shield)
            line_sensor_shield();
    }
}

robot(show_bottom_wall = false, show_middle_wall = false, show_other_components = false);
