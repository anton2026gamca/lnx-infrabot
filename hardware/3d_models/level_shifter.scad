$fn = 90;

module level_shifter_holes(
    hole_d = 3.3,
    hole_h = 300,
    hole_positions = [[16, -10], [-16, 10]]
) {
    for (pos = hole_positions) {
        translate([pos[0], pos[1], 0])
            cylinder(d = hole_d, h = hole_h, center = true);
    }
}

module level_shifter(
    body_size = [54, 25, 16],
    hole_d = 3.3,
    hole_h = 300,
    hole_positions = [[16, -10], [-16, 10]]
) {
    difference() {
        cube(body_size, center = true);
        level_shifter_holes(hole_d = hole_d, hole_h = hole_h, hole_positions = hole_positions);
    }
}

level_shifter();
