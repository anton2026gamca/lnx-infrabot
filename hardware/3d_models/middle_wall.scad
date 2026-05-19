$fn = 90;

use <robot_shared.scad>;

module middle_wall(usb_hole = 1) {
    difference() {
        union() {
            translate([0, 0, 55])
                for (angle = [90, 180, 270]) {
                    for (dir = [1, -1]) {
                        difference() {
                            translate([0, 0, 20])
                                platform_connection_holes(13, 40 - 0.2);
                            platform_connection_holes(7);
                        }
                        for (wall_dir = [1, -1]) {
                            translate([wall_dir * 89, 0, 20])
                                cube([2, 70, 40 - 0.2], center = true);
                        }
                        translate([0, 98, 20])
                            cube([80, 2, 40 - 0.2], center = true);
                        translate([0, 30, 20])
                            difference() {
                                scale([1, 0.855, 1])
                                    cylinder(h = 40 - 0.2, d = 180, center = true);
                                scale([1, 0.855, 1])
                                    cylinder(h = 130, d = 175, center = true);
                                translate([0, -50, 0])
                                    cube([200, 100, 100], center = true);
                                cube([80, 300, 100], center = true);
                            }
                    }
                }
        }
    }
}

middle_wall();
