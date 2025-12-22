$fn = 90;
use <Robot.scad>

module teensy_board_brackets_holes(diameter = 3.3, height = 200) {
    translate([-40,10,0])
       cylinder(h=height,d=diameter,center=true);translate([30,0,0])
       cylinder(h=height,d=diameter,center=true);
    translate([-30,-15,0])
       cylinder(h=height,d=diameter,center=true);
}
module teensy_board_holes(diameter, height) {
    translate([0,0,0])
       cylinder(h=height,d=diameter,center=true);translate([0,0,0])
       cylinder(h=height,d=diameter,center=true);
    translate([0,0,0])
       cylinder(h=height,d=diameter,center=true);
}
module teensy_board_bracket() {
    translate([0,0,-16.2]) difference() {
        union() {
            cube([100,50,2],center=true);
            translate([0,0,3.6])
                teensy_board_holes(6, 5);
        }
        teensy_board_holes(3.3, 200);
        teensy_board_brackets_holes(3.3, 200);
    }
}
module teensy_board() {
    difference() {
        cube([100,50,20],center=true);
        teensy_board_holes(3.3, 200);
    }
}
//teensy_board();
teensy_board_bracket();
