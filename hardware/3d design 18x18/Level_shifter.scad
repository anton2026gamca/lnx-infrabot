$fn = 90;

use <Robot.scad>;


module level_shifter_holes(diameter=3.3,height=300) {
    translate([-17,0,0])
        cylinder(d = diameter, h = height, center=true);
    for (A = [1, -1]) translate([35.56-17,A*23/2,0])
        cylinder(d = diameter, h = height, center=true);
}
module level_shifter() {
    difference() {
        cube([45, 31, 16],center=true);
        level_shifter_holes();
    }
}
level_shifter();