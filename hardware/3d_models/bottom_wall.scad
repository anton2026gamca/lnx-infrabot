$fn = 90;

use <robot_shared.scad>;

module wall_behind_wheel() {
    difference() {
        cube([2, 47, 40 - 0.2], center = true);
        translate([0, 0, -5])
            rotate([0, 90, 0])
                cylinder(d = 27, h = 200, center = true);
        translate([0, 0, -18.5])
            cube([10, 27, 30], center = true);
    }
}

module bottom_wall(scale_factor = [1, 1, 1]) {
    scale(scale_factor) {
        intersection() {
            cylinder(d=215, h=200, center=true);

            union() {
                for (dir = [1, -1]) {
                    difference() {
                        translate([0, 0, 33])
                            platform_connection_holes(12, 40 - 0.2);
                        platform_connection_holes(6.2);
                    }

                    translate([dir * 89, 0, 33])
                        cube([2, 86, 40 - 0.2], center = true);
                }

                intersection() {
                    translate([0, 98, 33])
                        cube([90, 8, 40 - 0.2], center = true);
                    for (angle = [-8, -4, 0, 4, 8]) {
                        rotate([0, 0, angle]) {
                            translate([0, 102, 33])
                                cube([90, 4, 40 - 0.2], center = true);
                        }
                    }
                }

                translate([0, 0, 33]) {
                    for (angle = [45:90:360]) {
                        difference() {
                            rotate([0, 0, angle]) {
                                translate([85, 0, 0])
                                    wall_behind_wheel();
                            }
                            translate([0, -(180 / 2 - 12 + 50), 0])
                                cube([100, 100, 100], center = true);
                            platform_connection_holes(6.2);
                        }
                    }
                    for (angle = [0, 180]) {
                        rotate([0, 0, angle]) {
                            translate([88, 0, 0]) {
                                for (side = [1, -1]) {
                                    translate([-5, side * (44), 0])
                                        cube([14, 2, 40 - 0.2], center = true);
                                }
                            }
                        }
                    }
                    for (angle = [90]) {
                        rotate([0, 0, angle]) {
                            translate([88, 0, 0]) {
                                for (side = [1, -1]) {
                                    translate([0, side * (44), 0])
                                        cube([23, 2, 40 - 0.2], center = true);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bottom_wall();
