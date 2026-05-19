$fn = 90;

module line_sensor_shield(
    outer_d = 100,
    inner_d = 95,
    outer_h = 5,
    inner_h = 7,
    post_size = [20, 10, 2],
    post_offset = [50, 0, -1],
    hole_d = 3.3,
    hole_h = 100
) {
    translate([0, 0, 8]) {
        difference() {
            union() {
                translate([0, 0, -5])
                    cylinder(d = outer_d, h = outer_h);
                for (dir = [1, -1]) {
                    translate([dir * post_offset[0], post_offset[1], post_offset[2]])
                        cube(post_size, center = true);
                }
            }
            translate([0, 0, -6])
                cylinder(d = inner_d, h = inner_h);
            for (dir = [1, -1]) {
                translate([dir * (post_offset[0] + 5), post_offset[1], post_offset[2]])
                    cylinder(d = hole_d, h = hole_h, center = true);
            }
        }
    }
}

line_sensor_shield();
