$fn = 90;

use <ir_sensor.scad>;

module uppest_part(
    height = 20,
    thickness = 2.5,
    ir_rotation_offset = 53,
    ir_height_offset = 15,
    ir_holders_d = 4,
    camera_holder_d = 9,
    camera_holder_offset = 5,
    camera_holder_thickness = 6,
    part_offset = [0, 0, 187],
    show_ir_sensor = true
) {
    translate(part_offset) {
        cube([154, thickness, height], center = true);

        if (show_ir_sensor)
            translate([0, 0, height / 2 + ir_height_offset])
                rotate([0, 0, ir_rotation_offset])
                    %ir_sensor();

        for (dir = [1, -1]) {
            rotate([0, 0, ir_rotation_offset]) {
                translate([0, 0, height / 2]) {
                    translate([dir * -7.5, dir * 10, 0]) {
                        difference() {
                            union() {
                                cylinder(d = ir_holders_d, h = ir_height_offset, center = false);
                                translate([0, 0, -4])
                                    cylinder(d1 = 0, d2 = ir_holders_d, h = 4, center = false);
                            }
                            translate([0, 0, 1])
                                cylinder(d = 1.9, h = ir_height_offset, center = false);
                        }
                    }
                }
            }

            translate([dir * 81.75, 0, 0]) {
                difference() {
                    cube([12.5, 15, height], center = true);
                    translate([dir * 1.25, 0, 0]) {
                        cube([11, 10.2, height + 1], center = true);
                        for (hole_dir = [-1, 1]) {
                            translate([0, 0, hole_dir * 5])
                                rotate([90, 90, 0]) {
                                    cylinder(d = 3.3, h = 100, center = true);
                                    cylinder(d = 6.4, h = 50, center = false, $fn = 6);
                                }
                        }
                    }
                }
            }

            for (rotation = [0, 180]) {
                translate([dir * (12.7 + camera_holder_thickness / 2), 0, camera_holder_d / 2 - height / 2]) {
                    rotate([0, 90, rotation]) {
                        translate([0, -camera_holder_offset - thickness / 2, 0]) {
                            difference() {
                                union() {
                                    cylinder(d = camera_holder_d, h = camera_holder_thickness, center = true);
                                    translate([0, camera_holder_offset / 2, 0])
                                        cube([camera_holder_d, camera_holder_offset, camera_holder_thickness], center = true);
                                }
                                cylinder(d = 3.3, h = 16, center = true);
                            }
                        }
                    }
                }
            }
        }
    }
}

uppest_part();
