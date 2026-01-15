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
    translate([0,0,wheel_d/2]) for (A = [0:90:359]) {
        rotate([A,90,45]) translate([0,0,wheel_offset - 5]) 
            cylinder(wheel_h + 10, d = wheel_d + 6);
            
    }
}
