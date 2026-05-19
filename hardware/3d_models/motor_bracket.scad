$fn = 90;

module wheel_connector(
    wheel_d = 55,
    wheel_offset = 90,
    wheel_angles = [0:90:359],
    wheel_rotation = 45,
    connector_h = 5,
    connector_d = 17.5,
    axle_hole_d = 3.5,
    axle_hole_h = 10
) {
    rotate([0, 0, wheel_rotation])
        translate([0, 0, wheel_d / 2])
            for (angle = wheel_angles) {
                rotate([angle, 90, 0])
                    translate([0, 0, wheel_offset - connector_h / 2]) {
                        difference() {
                            cylinder(h = connector_h, d = connector_d, center = true);
                            cylinder(h = axle_hole_h, d = axle_hole_d, center = true);
                        }
                    }
            }
}

module motor_bracket_holes(
    hole_d = 3.3,
    hole_h = 100,
    holes = [0:1:4],
    hole_set = 6.35,
    hole_offset = 6.4,
    wheel_offset = 90,
    wheel_angles = [0:90:359],
    rotation_offset = 45
) {
    for (angle = wheel_angles) {
        rotate([0, 0, angle + rotation_offset])
            translate([wheel_offset - 5, 0, 0]) {
                for (hole_idx = holes) {
                    rotate([0, 0, 180])
                        translate([hole_idx * hole_set + hole_offset, 0, 0])
                            cylinder(d = hole_d, h = hole_h, center = true);
                }
            }
    }
}

module motor_bracket(
    width = 25,
    thickness = 2,
    height = 27,
    length = 52,
    num_holes = 7,
    hole_set = 6.35,
    shaft_hole_d = 7.5,
    mount_hole_d = 3.3,
    mount_hole_h = 100,
    mount_offset = 8.5,
    mount_span = 14.5,
    slot_size = [20, 30, 25],
    slot_offset = [0, 0, 27],
    slot_clear_d = 25,
    slot_clear_offset = [0, 0, 14.5],
    side_hole_span = 21.7,
    side_slot_size = [3.3, 4, 100],
    side_slot_offset = 2
) {
    rotate([0, 0, 180])
        translate([0, 0, -14.5])
            difference() {
                union() {
                    translate([length / 2, 0, thickness / 2])
                        cube([length, width, thickness], center = true);
                    translate([thickness / 2, 0, height / 2])
                        cube([thickness, width, height], center = true);
                }
                for (dir = [-1, 1]) {
                    rotate([0, 90, 0])
                        translate([dir * mount_offset - mount_span, 0, 0])
                            cylinder(d = mount_hole_d, h = mount_hole_h, center = true);
                    rotate([90, 0, 90])
                        translate([dir * mount_offset, mount_span, 0])
                            cylinder(d = mount_hole_d, h = mount_hole_h, center = true);
                }
                translate(slot_clear_offset)
                    rotate([0, 90, 0])
                        cylinder(d = shaft_hole_d, h = mount_hole_h, center = true);
                difference() {
                    translate(slot_offset)
                        cube(slot_size, center = true);
                    translate(slot_clear_offset)
                        rotate([0, 90, 0])
                            cylinder(d = slot_clear_d, h = mount_hole_h, center = true);
                }
                for (idx = [0:1:num_holes - 1]) {
                    translate([idx * hole_set + 6.4, 0, 0])
                        cylinder(d = mount_hole_d, h = mount_hole_h, center = true);
                }
                for (idx = [0:1:num_holes - 2]) {
                    for (dir = [1, -1]) {
                        translate([idx * hole_set + 9.5, dir * side_hole_span / 2, 0]) {
                            cylinder(d = mount_hole_d, h = mount_hole_h, center = true);
                            translate([0, dir * side_slot_offset, 0])
                                cube(side_slot_size, center = true);
                        }
                    }
                }
            }
}

module motors(
    wheel_d = 55,
    wheel_offset = 90,
    wheel_angles = [0:90:359],
    wheel_rotation = 45,
    motor_length = 17,
    motor_body_d = 24.4,
    motor_body_extension = 30.8,
    shaft_d = 7,
    shaft_h = 2.5,
    tip_d = 4,
    tip_h = 10,
    mount_hole_d = 3.3,
    mount_offset = 8.5
) {
    translate([0, 0, wheel_d / 2])
        for (angle = wheel_angles) {
            rotate([angle, 90, wheel_rotation])
                translate([0, 0, wheel_offset - 3]) {
                    difference() {
                        union() {
                            translate([0, 0, -(motor_length + motor_body_extension) / 2 - shaft_h / 2 - tip_h / 2])
                                cylinder(h = motor_length + motor_body_extension, d = motor_body_d, center = true);
                            translate([0, 0, -shaft_h / 2 - tip_h / 2])
                                cylinder(h = shaft_h, d = shaft_d, center = true);
                            translate([0, 0, -tip_h / 2])
                                cylinder(h = tip_h, d = tip_d);
                        }
                        for (dir = [-1, 1]) {
                            translate([0, dir * mount_offset, -tip_h])
                                cylinder(d = mount_hole_d, h = tip_h);
                        }
                    }
                }
        }
}

module motor_brackets_all(
    wheel_d = 55,
    wheel_offset = 90,
    wheel_angles = [0:90:359],
    rotation = 0
) {
    translate([0, 0, wheel_d / 2])
        for (angle = wheel_angles) {
            rotate([0, rotation, angle + 45])
                translate([wheel_offset - 5, 0, 0])
                    motor_bracket();
        }
}

module motor_brackets_holders(
    length = 52,
    width = 25,
    holder_h = 11,
    wheel_angles = [45:90:359]
) {
    difference() {
        for (angle = wheel_angles) {
            rotate([0, 0, angle])
                translate([59, 0, 0])
                    cube([length, width, holder_h], center = true);
        }
        motor_bracket_holes();
    }
}

motor_bracket();
