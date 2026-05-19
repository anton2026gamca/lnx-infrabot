$fn = 90;

use <ball_zone.scad>;
use <robot_shared.scad>;
use <wheels.scad>;
use <motor_bracket.scad>;
use <motor_driver_bracket.scad>;
use <kicker.scad>;

module led_holes(diode_d = 7, height = 10, ring_radius = 45, angles = [0:15:359]) {
    for (angle = angles) {
        rotate([0, 0, angle])
            translate([0, ring_radius, 0])
                cylinder(d = diode_d, h = height, center = true);
    }
}

module bottom_part(
    robot_d = 215,
    wheel_d = 60,
    wheel_h = 12.5,
    wheel_offset = 90
) {
    intersection() {
        translate([0, 12, 0])
            rotate([0, 0, 0])
                cube([180, 180, 200], center = true);
        difference() {
            translate([0, 0, 12])
                union() {
                    cylinder(d = robot_d, h = 2, center = true);
                    for (dir = [1, -1]) {
                        translate([dir * 72, 0, 10])
                            cube([30, 39, 20], center = true);
                    }
                    difference() {
                        translate([0, 0, -4])
                            cylinder(d = robot_d, h = 4);
                    }
                }
            translate([0, 0, 10])
                led_holes();
            difference() {
                translate([0, 0, 0])
                    cylinder(d = 80, h = 200, center = true);
                translate([0, 0, 7])
                    difference() {
                        for (dir = [1, -1]) {
                            translate([dir * 42, 0, 1.5])
                                cube([20, 15, 3], center = true);
                            translate([0, dir * 35, 1.5])
                                cube([15, 20, 3], center = true);
                        }
                        for (dir = [1, -1]) {
                            translate([dir * 37, 0, 0])
                                cylinder(d = 3.3, h = 100, center = true);
                            translate([0, dir * 30, 0])
                                cylinder(d = 3.3, h = 100, center = true);
                        }
                    }
            }
            translate([0, 0, 14.5])
                cylinder(d = 100, h = 10, center = true);
            platform_connection_holes();
            for (dir = [1, -1]) {
                translate([dir * 55, 0, -1])
                    cylinder(d = 3.3, h = 100, center = true);
            }
            wheels_cutout(wheel_offset = wheel_offset);
            motor_bracket_holes(wheel_offset = wheel_offset);
            translate([0, 0, 8])
                motor_bracket_holes(hole_d = 6.4, hole_h = 4, holes = [0:1:4], wheel_offset = wheel_offset, $fn = 6);
            motor_driver_bracket_holes();
            ball_zone_holes();
            translate([0, -70, 23.1])
                kicker_holes();
            for (dir = [1, -1]) {
                translate([dir * 72, 0, 25])
                    cube([26, 35, 25], center = true);
            }
        }
    }
}

bottom_part();
