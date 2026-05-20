$fn = 90;

use <robot_shared.scad>;

module led(color = "yellow") {
    color(color) difference() {
        union() {
            sphere(d = 3);
            cylinder(d = 3, h = 3);
            translate([0,0,2.5])cylinder(d = 4, h = 1);
        }
        translate([2.5,0,0])cube([2,100,100], center=true);
    }
    color("grey") for(A = [1, -1])
        translate([A,0,3])cylinder(d = 0.5, h = 5);
}

module line_sensor() {
    difference() {
        union() {
            color("darkgreen")cylinder(d = 100, h = 2);
            for(A = [0:15:360]) {
                rotate([0,0,A])translate([-45,0,-5]) {
                     if (A % 10 == 0)
                         led("black");
                     else
                         led("white");
                }
            }
        }
        translate([0,0,4])cylinder(d = 110, h = 20);
    }
}
line_sensor();

