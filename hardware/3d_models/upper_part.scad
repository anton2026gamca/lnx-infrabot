$fn = 90;

use <robot_shared.scad>;

module upper_part(
    robot_d = 215,
    wheel_offset = 90
) {
    difference() {
        translate([0, 0, 96])
            union() {
                intersection() {
                    cylinder(d = robot_d, h = 2, center = true);
                    translate([0, 12, 0])
                        rotate([0, 0, 0])
                            cube([180, 180, 200], center = true);
                }
                translate([0, 100, 19])
                    difference() {}
            }
        platform_connection_holes();
        translate([0, -120, 100])
            scale([1.05, 1, 1])
                cylinder(h = 100, d = 200, center = true);
        for (angle = [225, 315]) {
            rotate([0, 0, angle]) {
                side = 45;
                inner_d = side / tan(180 / 8);
                translate([wheel_offset + inner_d / 2 - 4, 0, 50]) {
                    rotate([0, 0, 45])
                        cube([inner_d, inner_d, 100], center = true);
                }
            }
        }
        for (dir = [1, -1]) {
            translate([dir * 72, 0, 50])
                cube([26, 35, 100], center = true);
        }
        translate([-22.86 / 2, 15, 100])
            cube([2.54 + 0.5, 6 * 2.54 + 0.5, 100], center = true);
        translate([38 / 2, 40, 100])
            cube([2.54 + 0.5, 4 * 2.54 + 0.5, 100], center = true);
        translate([0, 53, 100])
            cube([(4 * 3 + 1) * 2.54 + 0.5, 2.54 + 0.5, 100], center = true);
        translate([50 + 8 * 2.54 / 2, 50, 100])
            rotate([0, 0, 45]) {
                cube([2.54 + 0.5, 6 * 2.54 + 0.5, 200], center = true);
                translate([7 * 2.54, 0, 0])
                    cube([2.54 + 0.5, 4 * 2.54 + 0.5, 200], center = true);
            }
        translate([-55, 65, 50])
            rotate([0, 0, 45])
                cube([19, 13, 100], center = true);
    }

    translate([0, 0, 96])
        difference() {
            for (dir = [1, -1]) {
                translate([dir * (16.7 - 1), -24.5, 3.5]) {
                    rotate([0, 90, 0])
                        cylinder(d = 9, h = 6, center = true);
                    translate([0, 5, -3.5])
                        cube([6, 8, 2], center = true);
                }
            }
            translate([0, -24.5, 3.5])
                rotate([0, 90, 0])
                    cylinder(d = 3.3, h = 80, center = true);
        }
}

upper_part();
