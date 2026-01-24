$fn = 90;
use <Robot.scad>
wheel_d = 55;
wheel_h = 12.5;
wheel_offset = 90;
robot_d = 215;
robot_h = 220;

ballDiameter = 43;

drv_x = 51;
drv_y = 25;
drv_z = 2;
pin_x = 2.55;
module driver() {
    cube([drv_x,drv_y,drv_z],center=true);
    for(A = [0, 47 - pin_x]) {
        translate([A - drv_x/2 + pin_x/2,0,-2]) {
            cube([pin_x,drv_y,drv_z],center=true);
        }
        translate([A - drv_x/2 + pin_x/2,0,-6]) {
            cube([0.65,drv_y,6],center=true);
        }
    }
    translate([8.7+30/2-drv_x/2,7.8/2-drv_y/2-0.4,6]) {
        cube([30,7.8,10],center = true);
        for (A = [0:1:5]) {
            translate([30/2-2.5-5*A,0,-8.75])
                cylinder(d = 3.6, h = 3.5,center=true);
        }
    }
}

module driver_holder() {
    difference() {
        union() {
            translate([0,92 - 75,-2])
                cylinder(d = 10, h = 2, center=true);
            for(A = [1, -1]) {
                translate([A*30,65 - 75,-2])
                    cylinder(d = 10, h = 2, center=true);
            }
            translate([0,0,-2]) {
                cube([drv_x+10,drv_y+10,drv_z],center=true);
            }
        }
        
        for(A = [0, 47 - pin_x]) {
            translate([A - drv_x/2 + pin_x/2,0,-2]) {
        cube([pin_x+0.6,drv_y+0.6,drv_z+1],center=true);
            }
        }
        translate([8.7+30/2-drv_x/2,7.8/2-drv_y/2-0.4,6]) {
            cube([30,7.8,10],center = true);
            for (A = [0:1:5]) {
                translate([30/2-2.5-5*A,0,-8])
                    cylinder(d = 3.6 + 4, h = 3.5,center=true);
            }
        }
        translate([0,92 - 75,-2])
            cylinder(d = 3.3, h = 100, center=true);
        for(B = [1, -1]) {
            translate([B*30,65 - 75,-2])
                cylinder(d = 3.3, h = 100, center=true);
        }
    }
    
}
module motor_driver_brackets_all() {
    for(A = [90, 270]) {
        rotate([0,0,A])translate([0,75,14+25])driver_holder();
    }
}
module motor_driver_bracket_holes() {
    translate([0,92,100])
        cylinder(d = 3.3, h = 200, center=true);
    for(B = [1, -1]) {
        translate([B*30,65,100])
            cylinder(d = 3.3, h = 200, center=true);
    
    }
}
%driver();
driver_holder();
%motor_driver_bracket_holes();