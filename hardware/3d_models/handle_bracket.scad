$fn = 90;

use <robot_shared.scad>;

module handle_bracket(scale_factor = [1, 1, 1]) {
    scale(scale_factor)
        for (rotation = [0, 180]) {
            rotate([0, 0, rotation]) {
                difference() {
                    union() {
                        translate([83, 0, 99])
                            cube([12, 85, 3], center = true);
                        translate([83, 0, 105])
                            cube([12, 60.81, 11.33], center = true);
                        for (dir = [1, -1]) {
                            translate([0, 0, 134])
                                rotate([45 * dir, 0, 0]) {
                                    translate([83, dir * -17, 0])
                                        cube([12, 42, 10], center = true);
                                }
                        }
                        translate([83, 0, 135])
                            cube([12, 15, 30], center = true);
                    }
                    platform_connection_holes();
                    for (dir = [1, -1]) {
                        translate([dir * 72, 0, 70])
                            cube([26, 35, 100], center = true);
                        translate([dir * 83, 0, 110])
                            cube([11, 10.2, 100], center = true);
                        for (height = [10, 0]) {
                            translate([dir * 83, 0, height + 135])
                                rotate([90, 0, 0])
                                    cylinder(d = 3.3, h = 200, center = true);
                        }
                    }
                    for (height = [10, 0]) {
                        rotate([90, 0, 0])
                            translate([83, height + 135, rotation == 0 ? -20 : 20])
                                cylinder(d = 6.4, h = 50, center = true, $fn = 6);
                    }
                }
            }
        }
}

handle_bracket();
