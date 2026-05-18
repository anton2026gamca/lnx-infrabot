$fn = 90;
wheel_d = 55;
wheel_h = 12.5;
wheel_offset = 90;
robot_d = 215;
robot_h = 220;

ballDiameter = 43;
use <Robot.scad>

module wheels() {
    translate([0,0,wheel_d/2]) 
    for (A = [0:90:359]){
        rotate([A,90,45]) translate([0,0,wheel_offset]) {
            difference() {
                cylinder(wheel_h, d = wheel_d);
                cylinder(h = 10, d = 3.5, center=true);
            }
        }
    }
    
}

module wheels_cutout() {
    for (angle = [45, 135, 225, 315]) {
        rotate([0, 0, angle]) {
            side = 50;
            d_outer = side / sin(180 / 8);
            d_inner = side / tan(180 / 8);
            translate([wheel_offset + d_inner / 2 - 4, 0, 0]) {
                rotate([0, 0, 360 / 16])
                    cylinder(100, d=d_outer, $fn=8);
            }
        } 
    }
}
wheels_cutout();