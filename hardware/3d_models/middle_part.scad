$fn = 90;

use <robot_shared.scad>;
use <ball_zone.scad>
use <wheels.scad>;
use <motor_driver_bracket.scad>;
use <level_shifter.scad>;
use <raspberry_pi.scad>;

module middle_part(
    robot_d = 215,
    wheel_offset = 90,
    brackets_support = 1,
    rpi_pos = [0, -30, 55],
    level_shifter_pos = [-35, 32, 57.4],
    show_raspberry = true
) {
    intersection() {
        translate([0, 12, 0])
            rotate([0, 0, 0])
                cube([180, 180, 200], center = true);
        difference() {
            union() {
                translate([0, 0, 55 / 2 - 15.5 + 42])
                    cylinder(d = robot_d, h = 2, center = true);
                if (brackets_support) {
                    translate(rpi_pos)
                        raspberry_holes(6, 5);
                    translate(level_shifter_pos)
                        level_shifter_holes(6, 5);
                }
            }
            translate(rpi_pos) {
                translate([0, 0, -10])
                    raspberry_holes();
                if (show_raspberry)
                    %raspberry();
            }
            translate(level_shifter_pos)
                level_shifter_holes();
            translate([0, 70, 0])
                cube([50, 35, 200], center = true);
            platform_connection_holes();
            motor_driver_bracket_holes();
            ball_zone_holes();
            wheels_cutout(wheel_offset = wheel_offset);
            translate([-40, 37, 70])
                level_shifter_holes();
            cylinder(d = 30, h = 200, center = true);
            for (dir = [1, -1]) {
                translate([dir * 72, 0, 23])
                    cube([26, 35, 100], center = true);
            }
        }
    }
}

middle_part();
