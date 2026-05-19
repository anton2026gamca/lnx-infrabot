$fn = 90;

module motor_driver_board(
    board_size = [51, 25, 2],
    pin_width = 2.55,
    pin_tail_width = 0.65,
    pin_offsets = [0, 47 - 2.55],
    pin_body_z = -2,
    pin_tail_z = -6,
    pin_tail_h = 6,
    connector_size = [30, 7.8, 10],
    connector_offset = [8.7 + 30 / 2 - 51 / 2, 7.8 / 2 - 25 / 2 - 0.4, 6],
    connector_hole_d = 3.6,
    connector_hole_h = 3.5,
    connector_hole_spacing = 5,
    connector_hole_count = 6,
    connector_hole_z = -8.75
) {
    cube(board_size, center = true);
    for (offset = pin_offsets) {
        translate([offset - board_size[0] / 2 + pin_width / 2, 0, pin_body_z]) {
            cube([pin_width, board_size[1], board_size[2]], center = true);
        }
        translate([offset - board_size[0] / 2 + pin_width / 2, 0, pin_tail_z]) {
            cube([pin_tail_width, board_size[1], pin_tail_h], center = true);
        }
    }
    translate(connector_offset) {
        cube(connector_size, center = true);
        for (idx = [0:1:connector_hole_count - 1]) {
            translate([connector_size[0] / 2 - 2.5 - connector_hole_spacing * idx, 0, connector_hole_z])
                cylinder(d = connector_hole_d, h = connector_hole_h, center = true);
        }
    }
}

module motor_driver_bracket(
    board_size = [51, 25, 2],
    bracket_clearance = 10,
    bracket_z = -2,
    mount_post_d = 10,
    mount_post_h = 2,
    mount_post_positions = [[0, 17], [30, -10], [-30, -10]],
    pin_width = 2.55,
    pin_clearance = 0.6,
    pin_offsets = [0, 47 - 2.55],
    pin_clearance_z = -2,
    pin_clearance_h = 3,
    connector_size = [30, 7.8, 10],
    connector_offset = [8.7 + 30 / 2 - 51 / 2, 7.8 / 2 - 25 / 2 - 0.4, 6],
    connector_hole_d = 3.6,
    connector_hole_h = 3.5,
    connector_hole_spacing = 5,
    connector_hole_count = 6,
    connector_hole_clearance = 4,
    connector_hole_z = -8,
    mount_hole_d = 3.3,
    mount_hole_h = 100
) {
    difference() {
        union() {
            for (pos = mount_post_positions) {
                translate([pos[0], pos[1], bracket_z])
                    cylinder(d = mount_post_d, h = mount_post_h, center = true);
            }
            translate([0, 0, bracket_z])
                cube([board_size[0] + bracket_clearance, board_size[1] + bracket_clearance, board_size[2]], center = true);
        }

        for (offset = pin_offsets) {
            translate([offset - board_size[0] / 2 + pin_width / 2, 0, pin_clearance_z]) {
                cube(
                    [pin_width + pin_clearance, board_size[1] + pin_clearance, board_size[2] + pin_clearance_h],
                    center = true
                );
            }
        }
        translate(connector_offset) {
            cube(connector_size, center = true);
            for (idx = [0:1:connector_hole_count - 1]) {
                translate([connector_size[0] / 2 - 2.5 - connector_hole_spacing * idx, 0, connector_hole_z])
                    cylinder(d = connector_hole_d + connector_hole_clearance, h = connector_hole_h, center = true);
            }
        }
        for (pos = mount_post_positions) {
            translate([pos[0], pos[1], bracket_z])
                cylinder(d = mount_hole_d, h = mount_hole_h, center = true);
        }
    }
}

module motor_driver_brackets(
    positions = [[0, 75, 30], [0, 75, 45]]
) {
    for (pos = positions) {
        translate(pos)
            motor_driver_bracket();
    }
}

module motor_driver_bracket_holes(
    hole_d = 3.3,
    hole_h = 200,
    mount_post_positions = [[0, 17], [30, -10], [-30, -10]],
    hole_z = 100,
    y_offset = 75
) {
    for (pos = mount_post_positions) {
        translate([pos[0], pos[1] + y_offset, hole_z])
            cylinder(d = hole_d, h = hole_h, center = true);
    }
}

motor_driver_bracket();
