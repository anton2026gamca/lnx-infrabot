$fn = 90;

use <bottom_part.scad>;
use <robot_shared.scad>;


module ball_zone_holes(
    diameter = 3.3,
    height = 300,
    hole_offset = [36, -73, 100]
) {
    for (dir = [1, -1]) {
        translate([dir * hole_offset[0], hole_offset[1], hole_offset[2]])
            cylinder(d = diameter, h = height, center = true);
    }
}

module ball_zone_cutout(
    robot_d = 215,
    cutout_size = [100, 30, 100],
    cutout_offset = [0, 0, 0],
    relief_size = [80, 30, 100],
    relief_offset = [37, 30, 0],
    relief_rotation = 45
) {
    translate([cutout_offset[0], -robot_d / 2 + 10, cutout_offset[2]])
        difference() {
            cube(cutout_size, center = true);
            for (dir = [1, -1]) {
                translate([-dir * relief_offset[0], relief_offset[1], relief_offset[2]])
                    rotate([0, 0, relief_rotation * dir])
                        cube(relief_size, center = true);
            }
        }
}

module ball_zone(
    robot_d = 215,
    robot_h = 220,
    wheel_d = 60,
    wheel_h = 12.5,
    wheel_offset = 90,
    zone_z = 33
) {
    intersection() {
        cylinder(d = robot_d, h = robot_h);
        translate([0, 0, zone_z]) {
            difference() {
                union() {
                    translate([0, -90, -2.6])
                        cube([100, 30, 45 - 0.4], center = true);
                    translate([0, 0, -100 - 0.2])
                        ball_zone_holes(12, 40 - 0.2);
                }
                translate([0, -106.713, 0])
                    cube([110, 10, 100], center = true);
                ball_zone_cutout(robot_d = robot_d);
                for (dir = [-1, 1]) {
                    rotate([0, 0, 180 + dir * 45])
                        translate([dir * -8.6, 84, 0])
                            cube([65, 4, 80], center = true);
                }
                translate([0, 0, -33]) {
                    translate([0, 0, wheel_d / 2])
                        for (angle = [0:90:359]) {
                            rotate([angle, 90, 45])
                                translate([0, 0, wheel_offset - 5])
                                    cylinder(wheel_h + 10, d = wheel_d + 6);
                        }
                }
                ball_zone_holes(5.6);
                translate([0, 12, -22.5]) {
                    cube([180, 180, 5], center = true);
                }
            }
        }
    }
}


ball_zone();
