$fn = 90;
wheel_d = 55;
wheel_h = 12.5;
wheel_offset = 90;
robot_d = 215;
robot_h = 220;

use <Wheels.scad>
use <Robot.scad>


module ball_zone_holes(diameter = 3.3, height = 300) {
    for (A = [1, -1]) {
        translate([A*36,-73,100])
            cylinder(d = diameter, h = height, center=true);
    }
}
module ball_zone_cutout() {
    translate([0,-robot_d/2 + 10,0]) difference () {
        cube([100,30,100],center=true);
        for (A = [1, -1]) {
            translate([-A*37,30,0])rotate([0,0, 45*A])
                cube([80,30,100],center=true);
        }
    }
}
module kicker_cutout() {
    translate([0,-76.5,23.15]) difference() {
        cube([55,12,20-0.4],center=true);
        rotate([90,0,0]) {
            color("red", 0.1)cylinder(d = 3.3, h = 30, center=true);
        }
        translate([0,2,8])
            cube([5.5,3.1,15],center=true);
        translate([0,2,0])rotate([90,30,0])
            cylinder(h = 3.1,d = 6.3,center=true,$fn=6);
        for (A = [1, -1]) {
            translate([A*70,20,0])scale([3,1,1])
                cylinder(h = 100, d = 50, center=true);
        }
    }
}
module ball_zone() {
    intersection() {
        color("white")MAXsize();
        translate([0,0,33]) difference() {
            union() {
                translate([0,-90,0]) cube([100,30,40-0.2],center=true);
                translate([0,0,-100])ball_zone_holes(12, 40-0.2);
            }
            // kicker cotout
            translate([0,-60,-10]) {
                cube([57,100,21],center=true);
            }
            
            translate([0,-106.713,0])
                cube([110,10,100],center=true);
            ball_zone_cutout();
            for (A=[45:90:360]) {
                rotate([0,0,A])translate([0,84,0])cube([65,4,80],center=true);
            }
            translate([0,0,-33])wheels_cutout();
            ball_zone_holes(5.6);
            
        }
    }
}
module kicker_holes() {
    rotate([0,0,-90])translate([-38.7/2,0,0])
        for( Y = [7, -7]) for (X = [0, -22]) {
            translate([X + 38.7/2 - 12, Y, 0])
            cylinder(d = 3.3, h = 200,center=true);
    }
}
module kicker() {
    rotate([0,0,-90])translate([-38.7/2,0,0])difference() {
        union() {
            cube([38.7,26,20], center=true);
            translate([38.7/2,0,0])rotate([0,90,0])
            cylinder(h = 19.6-13, d = 3);
        }
        translate([38.7/2,0,0])rotate([0,0,90])kicker_holes();
    }
}
ball_zone();
kicker_cutout();
translate([0,-70,23.1])kicker();
//translate([0,-110,0])ball();