$fn = 90;

module camera_holes(
    hole_d = 2.3,
    hole_h = 200,
    base_offset = [-10.5, 10, 0],
    hole_offsets = [[21, -12.5], [21, 0], [0, -12.5], [0, 0]]
) {
    translate(base_offset)
        for (pos = hole_offsets) {
            translate([pos[0], pos[1], 0])
                cylinder(d = hole_d, h = hole_h, center = true);
        }
}

module camera_bracket(
    camera_pos = [0, 0, 0],
    camera_angle = [0, 0, 0],
    base_size = [25, 24, 2],
    base_offset = [0, 0, -5],
    standoff_offset = [0, 0, -3.12],
    bracket_offsets = [8.5, 5, -9],
    holder_d = 9,
    holder_h = 8,
    hole_d = 3.3,
    hole_h = 80,
    hex_d = 6.4,
    hex_h = 12.5
) {
    translate(camera_pos)
        rotate(camera_angle)
            translate([0, 15.5, -5]) {
                difference() {
                    rotate([90, 0, 0]) {
                        translate(base_offset)
                            cube(base_size, center = true);
                        translate(standoff_offset)
                            camera_holes(4, 3);
                        for (dir = [1, -1]) {
                            for (height = [5, -9]) {
                                translate([dir * bracket_offsets[0], height, bracket_offsets[2]])
                                    rotate([0, 90, 0])
                                        cylinder(d = holder_d, h = holder_h, center = true);
                            }
                        }
                    }
                    for (height = [5, -9]) {
                        translate([0, 9, height]) {
                            rotate([0, 90, 0]) {
                                cylinder(d = hole_d, h = hole_h, center = true);
                                cylinder(d = hex_d, h = hex_h, center = true, $fn = 6);
                            }
                        }
                    }
                    rotate([90, 0, 0])
                        camera_holes();
                    translate([0, 10, 0])
                        rotate([90, 0, 0])
                            camera_holes(4.5, 8, $fn = 6);
                }
            }
}

camera_bracket();
