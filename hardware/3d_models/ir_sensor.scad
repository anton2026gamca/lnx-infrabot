$fn = 90;

module ir_sensor_bracket_holes(
    hole_d = 3.3,
    hole_height = 200,
    hole_offset = 8,
    angles = [0:90:359]
) {
    for (angle = angles) {
        rotate([0, 0, angle])
            translate([hole_offset, 0, 0])
                cylinder(d = hole_d, h = hole_height, center = true);
    }
}

module ir_sensor_holes(
    hole_d = 2,
    hole_height = 200,
    hole_positions = [[-7.5, 10], [7.5, 10], [7.5, -10]]
) {
    for (pos = hole_positions) {
        translate([pos[0], pos[1], 0])
            cylinder(d = hole_d, h = hole_height, center = true);
    }
}

module ir_sensor_bracket(
    base_d = 42,
    base_h = 2,
    base_z_offset = -11,
    holder_d = 8,
    holder_h = 10,
    holder_offset = 15.5,
    holder_z_offset = -6,
    holder_clear_d = 7,
    holder_clear_h = 100,
    holder_slot_size = [3, 10, 100],
    holder_slot_offset = [0, 5, 0],
    cable_hole_d = 10,
    cable_hole_h = 100,
    bracket_hole_d = 3.3,
    bracket_hole_h = 200,
    sensor_hole_d = 2,
    sensor_hole_h = 200
) {
    difference() {
        union() {
            translate([0, 0, base_z_offset])
                cylinder(d = base_d, h = base_h, center = true);
            for (angle = [0:60:359]) {
                rotate([0, 0, angle])
                    translate([0, holder_offset, holder_z_offset])
                        difference() {
                            cylinder(h = holder_h, d = holder_d, center = true);
                            cylinder(h = holder_clear_h, d = holder_clear_d, center = true);
                            translate(holder_slot_offset)
                                cube(holder_slot_size, center = true);
                        }
            }
        }
        cylinder(d = cable_hole_d, h = cable_hole_h, center = true);
        ir_sensor_bracket_holes(hole_d = bracket_hole_d, hole_height = bracket_hole_h);
        ir_sensor_holes(hole_d = sensor_hole_d, hole_height = sensor_hole_h);
    }
}

module ir_sensor(model_path = "mrm-ir-finder3.stl", model_offset = [-21, -21, 0]) {
    translate(model_offset)
        import(model_path);
}

ir_sensor_bracket();
