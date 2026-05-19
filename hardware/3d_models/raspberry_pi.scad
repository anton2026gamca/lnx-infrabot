$fn = 90;

module raspberry_bracket_support(
    support_d = 8,
    support_h = 5,
    hole_d = 3.3,
    hole_h = 200
) {
    difference() {
        cylinder(d = support_d, h = support_h, center = true);
        cylinder(h = hole_h, d = hole_d, center = true);
    }
}

module raspberry_bracket_support_all(
    support_d = 6,
    support_h = 5,
    hole_d = 3.3,
    hole_h = 200,
    side_offset = [-30, 20]
) {
    difference() {
        union() {
            cylinder(h = support_h, d = support_d, center = true);
            for (dir = [1, -1]) {
                translate([side_offset[0], dir * side_offset[1], 0])
                    cylinder(h = support_h, d = support_d, center = true);
            }
        }
        cylinder(h = hole_h, d = hole_d, center = true);
        for (dir = [1, -1]) {
            translate([side_offset[0], dir * side_offset[1], 0])
                cylinder(h = hole_h, d = hole_d, center = true);
        }
    }
}

module raspberry_bracket_holes(
    hole_d = 3.3,
    hole_h = 200,
    side_offset = [-30, 20]
) {
    cylinder(h = hole_h, d = hole_d, center = true);
    for (dir = [1, -1]) {
        translate([side_offset[0], dir * side_offset[1], 0])
            cylinder(h = hole_h, d = hole_d, center = true);
    }
}

module raspberry_holes(
    hole_d = 2.8,
    hole_h = 100,
    hole_positions = [[-39.0, -24.5], [-39.0, 24.5], [19.0, -24.5], [19.0, 24.5]]
) {
    for (pos = hole_positions) {
        translate([pos[0], pos[1], 0])
            cylinder(d = hole_d, h = hole_h, center = false);
    }
}

module raspberry_bracket(
    bracket_size = [85, 56, 2],
    hole_d = 2.8,
    hole_h = 100,
    standoff_d = 6,
    standoff_h = 5,
    hole_clear_h = 200,
    support_offset = [0, 0, -10]
) {
    difference() {
        union() {
            cube(bracket_size, center = true);
            raspberry_holes(hole_d = standoff_d, hole_h = standoff_h);
        }
        raspberry_bracket_holes(hole_h = hole_clear_h);
        translate(support_offset)
            raspberry_holes(hole_d = hole_d, hole_h = hole_h);
    }
}

module raspberry(model_path = "raspberry_pi_5.stl") {
    translate([-85.5 / 2, 56 / 2, 3])
        rotate([90, 0, 0])
            import(model_path);
}

module raspberry_with_everything(
    bracket_offset = [0, 0, -14.7],
    support_offset = [0, 0, -18.5],
    side_offset = [-30, 20]
) {
    %raspberry();
    translate(bracket_offset)
        raspberry_bracket();
    translate(support_offset)
        raspberry_bracket_support();
    for (dir = [1, -1]) {
        translate([side_offset[0], dir * side_offset[1], support_offset[2]])
            raspberry_bracket_support();
    }
}

raspberry_bracket();
