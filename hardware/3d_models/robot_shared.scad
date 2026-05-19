$fn = 90;


module platform_connection_holes(
    diameter = 3.3,
    height = 500,
    start_angle = 247.5,
    end_angle = 472.5,
    angle_step = 45,
    inner_offset = 90,
    outer_offset = 100
) {
    for (angle = [start_angle:angle_step:end_angle]) {
        if (angle > 320 && angle < 400) {
            rotate([0, 0, angle])
                translate([0, outer_offset, 0])
                    cylinder(d = diameter, h = height, center = true);
        } else {
            rotate([0, 0, angle])
                translate([0, inner_offset, 0])
                    cylinder(d = diameter, h = height, center = true);
        }
    }
}
