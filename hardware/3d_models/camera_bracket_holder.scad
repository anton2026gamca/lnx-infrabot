$fn = 90;

use <ir_sensor.scad>;

module camera_bracket_holder(
    base_size = [41.4, 41, 2],
    base_offset = [0, -5, 0],
    holder_d = 9,
    holder_h = 8,
    holder_offsets = [16.7, -25.5, 3.5],
    ir_hole_offset = [0, 0, 0],
    slot_size = [25.4, 20, 100],
    slot_offset = [0, -29, 0],
    bolt_offset = [0, -25.5, 3.5],
    bolt_d = 3.3,
    bolt_h = 80,
    outer_d = 44,
    outer_scale = [1, 2.1, 1],
    outer_offset = [0, -35, 0]
) {
    translate([0, 50, 0])
        intersection() {
            difference() {
                union() {
                    translate(base_offset)
                        cube(base_size, center = true);
                    for (dir = [1, -1]) {
                        translate([dir * holder_offsets[0], holder_offsets[1], holder_offsets[2]])
                            rotate([0, 90, 0])
                                cylinder(d = holder_d, h = holder_h, center = true);
                    }
                }
                translate(ir_hole_offset)
                    ir_sensor_bracket_holes();
                translate(slot_offset)
                    cube(slot_size, center = true);
                translate(bolt_offset)
                    rotate([0, 90, 0])
                        cylinder(d = bolt_d, h = bolt_h, center = true);
            }
            translate(outer_offset)
                scale(outer_scale)
                    cylinder(d = outer_d, h = 100, center = true);
        }
}

camera_bracket_holder();
