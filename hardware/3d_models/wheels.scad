$fn = 90;

module wheels(
    wheel_d = 60,
    wheel_h = 12.5,
    wheel_offset = 90,
    wheel_z_offset = 55 / 2,
    axle_hole_d = 3.5,
    axle_hole_h = 10,
    wheel_rotation = 45,
    wheel_angles = [0:90:359]
) {
    translate([0, 0, wheel_z_offset])
        for (angle = wheel_angles) {
            rotate([angle, 90, wheel_rotation])
                translate([0, 0, wheel_offset]) {
                    difference() {
                        cylinder(wheel_h, d = wheel_d);
                        cylinder(h = axle_hole_h, d = axle_hole_d, center = true);
                    }
                }
        }
}

module wheels_cutout(
    wheel_offset = 90,
    cutout_side = 45,
    cutout_height = 100,
    corner_offset = 4,
    rotation_offset = 360 / 16,
    angles = [45, 135, 225, 315]
) {
    for (angle = angles) {
        rotate([0, 0, angle]) {
            outer_d = cutout_side / sin(180 / 8);
            inner_d = cutout_side / tan(180 / 8);
            translate([wheel_offset + inner_d / 2 - corner_offset, 0, 0]) {
                rotate([0, 0, rotation_offset])
                    cylinder(cutout_height, d = outer_d, $fn = 8);
            }
        }
    }
}

wheels();
