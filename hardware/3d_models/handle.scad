$fn = 90;

module handle(scale_factor = [1, 1, 1]) {
    scale(scale_factor)
        translate([0, 0, 0])
            difference() {
                union() {
                    for (dir = [1, -1]) {
                        translate([dir * 83, 0, 170 + 7.5])
                            cube([10, 10, 80 + 15], center = true);
                        translate([dir * 60, 0, 231 + 15])
                            rotate([0, dir * -45, 0])
                                cube([10, 10, 68], center = true);
                    }
                    translate([0, 0, 253.5 + 15])
                        cube([80, 10, 10], center = true);
                }
                for (dir = [1, -1]) {
                    for (height = [10, 0]) {
                        translate([dir * 83, 0, height + 135])
                            rotate([90, 0, 0])
                                cylinder(d = 3.3, h = 200, center = true);
                    }
                }
                for (dir = [1, -1]) {
                    for (height = [0:5:60]) {
                        translate([dir * 83, 0, height + 157])
                            rotate([90, 0, 0])
                                cylinder(d = 3.3, h = 100, center = true);
                    }
                }
            }
}

handle();
