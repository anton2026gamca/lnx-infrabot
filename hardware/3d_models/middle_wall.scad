$fn = 90;

use <robot_shared.scad>;



module middle_wall(robot_d = 215) {
    translate([0, 0, 42])intersection() {
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
                for (angle = [45, 135]) {
                    difference() {
                        rotate([0, 0, angle]) {
                            translate([85, 0, -17])
                                cube([2, 47, 5.8], center = true);
                            translate([0,0,-10]) {
                                translate([100, 0, 10])rotate([0,45,0])
                                    cube([2, 44, 41], center = true);
                                rotate([0,0,-angle])for (A = [1, -1]) {
                                    translate([A*55,90,6])rotate([0,A*45,0])
                                        cube([2, 28, 30], center = true);
                                    translate([A*83,55,6])rotate([-45,0,0])
                                        cube([14, 2, 30], center = true);
                                }
                            }
                        }
                        rotate([0,0,45])translate([100,0,0])
                            cube([100,12,5], center = true);
                        translate([0,0,-10+33-30])for (A = [1, -1]) {
                            translate([A*55-A*25/2,90,5.5+25/2])rotate([0,A*45,0])
                                cube([2 + 35, 28, 40], center = true);
                            translate([A*83,55-20/2,4.5+20/2])rotate([-45,0,0])
                                cube([14, 2 + 27, 40], center = true);
                        }
                        // Does not work
                        rotate([0, 0, angle])
                            translate([100 - 20/2,0,10+20/2])rotate([0,45,0])
                                cube([2 + 27, 45, 42], center = true);

                        translate([0, -(180 / 2 - 12 + 50), 0])
                            cube([100, 100, 100], center = true);
                        platform_connection_holes(6.2);
                        
                        
                        
                    }
                }
                for (angle = [0, 180]) {
                    rotate([0, 0, angle]) {
                        translate([88, 0, 0]) {
                            for (side = [1, -1]) {
                                if ((side == -1 && angle == 0) || 
                                    (side == 1 && angle == 180))
                                    translate([-5, side * (44), 0])
                                        cube([14, 2, 40 - 0.2], center = true);
                                else
                                    translate([-5, side * (44), -17])
                                        cube([14, 2, 5.8], center = true);
                            
                            }
                        }
                    }
                }
                for (angle = [90]) {
                    rotate([0, 0, angle]) {
                        translate([88, 0, 0]) {
                            for (side = [1, -1]) {
                                translate([0, side * (44), -17])
                                    cube([23, 2, 5.8], center = true);
                            }
                        }
                    }
                }
            }
            intersection() {
                difference() {
                    intersection() {
                        translate([0,0,33])
                            cylinder(d = robot_d, h = 40 - 0.2, center=true);
                        translate([0, 12, 0])
                            cube([180, 180, 200], center=true);
                    }
                    intersection() {
                        cylinder(d = robot_d - 2, h = 210, center=true);
                        translate([0, 12, 0])
                            cube([180 - 2, 180 - 2, 210], center=true);
                    }
                }
                translate([0,0,1])for (angle = [45, 135]) {
                    translate([0,0,-10+33])rotate([0, 0, angle]) {
                        translate([100 - 20/2 , 0, 10 + 20/2])rotate([0,45,0])
                            cube([2 + 27, 45, 42], center = true);
                        rotate([0,0,-angle])for (A = [1, -1]) {
                            translate([A*55-A*25/2,90,6+25/2])rotate([0,A*45,0])
                                cube([2 + 35, 28, 40], center = true);
                            translate([A*83,55-20/2,6+20/2])rotate([-45,0,0])
                                cube([14, 2 + 27, 40], center = true);
                        }
                    }
                }
                
            }
        }
    }
}

middle_wall();
