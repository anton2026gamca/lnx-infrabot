$fn = 90;

module kicker_holes(
    hole_d = 3.3,
    hole_height = 200,
    body_width = 38.7,
    hole_x_offsets = [0, -22],
    hole_y_offsets = [7, -7],
    hole_x_shift = 12
) {
    rotate([0, 0, -90])
        translate([-body_width / 2, 0, 0])
            for (y = hole_y_offsets)
                for (x = hole_x_offsets) {
                    translate([x + body_width / 2 - hole_x_shift, y, 0])
                        cylinder(d = hole_d, h = hole_height, center = true);
                }
}

module kicker(
    body_size = [38.7, 26, 20],
    pin_d = 3,
    pin_h = 19.6 - 13,
    hole_d = 3.3,
    hole_height = 200,
    hole_x_offsets = [0, -22],
    hole_y_offsets = [7, -7],
    hole_x_shift = 12
) {
    rotate([0, 0, -90])
        translate([-body_size[0] / 2, 0, 0])
            difference() {
                union() {
                    cube(body_size, center = true);
                    translate([body_size[0] / 2, 0, 0])
                        rotate([0, 90, 0])
                            cylinder(h = pin_h, d = pin_d);
                }
                rotate([0, 0, 90])
                    kicker_holes(
                        hole_d = hole_d,
                        hole_height = hole_height,
                        body_width = body_size[0],
                        hole_x_offsets = hole_x_offsets,
                        hole_y_offsets = hole_y_offsets,
                        hole_x_shift = hole_x_shift
                    );
            }
}

module kicker_cutout(
    cutout_offset = [0, -76.5, 23.15],
    cutout_size = [55, 12, 20 - 0.4],
    hole_d = 3.3,
    hole_h = 30,
    slot_size = [5.5, 3.1, 15],
    slot_offset = [0, 2, 8],
    hex_d = 6.3,
    hex_h = 3.1,
    hex_offset = [0, 2, 0],
    hex_rotation = [90, 30, 0],
    relief_offset = [70, 20, 0],
    relief_scale = [3, 1, 1],
    relief_d = 50,
    relief_h = 100
) {
    translate(cutout_offset)
        difference() {
            cube(cutout_size, center = true);
            rotate([90, 0, 0])
                cylinder(d = hole_d, h = hole_h, center = true);
            translate(slot_offset)
                cube(slot_size, center = true);
            translate(hex_offset)
                rotate(hex_rotation)
                    cylinder(h = hex_h, d = hex_d, center = true, $fn = 6);
            for (dir = [1, -1]) {
                translate([dir * relief_offset[0], relief_offset[1], relief_offset[2]])
                    scale(relief_scale)
                        cylinder(h = relief_h, d = relief_d, center = true);
            }
        }
}

kicker();
