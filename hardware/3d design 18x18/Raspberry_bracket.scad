$fn = 90;
use <Robot.scad>
module raspberry_bracket_support() {
    difference() {
        cylinder(d = 8, h = 5,center=true);
        cylinder(h = 200, d = 3.3, center=true);
    }
}
module raspberry_bracket_support_all() {
    difference() {
        union() {
            cylinder(h = 5, d = 6, center=true);
            for(A = [1, -1]) {
                translate([-30,A*20,0])
                    cylinder(h = 5, d = 6, center=true);
            }
        }
        cylinder(h = 200, d = 3.3, center=true);
        for(A = [1, -1]) {
            translate([-30,A*20,0])
                cylinder(h = 200, d = 3.3, center=true);
        }
    }
}
module raspberry_bracket_holes() {
    cylinder(h = 200, d = 3.3, center=true);
    for(A = [1, -1]) {
        translate([-30,A*20,0])
            cylinder(h = 200, d = 3.3, center=true);
    }
}
module raspberry_holes(dia = 2.8, height = 100) {
    translate([	-39.0,-24.5,0])cylinder(d = dia, h = height, center=false);
    translate([-39.0,+24.5,0])cylinder(d = dia, h = height, center=false);
    translate([+19.0,-24.5,0])cylinder(d = dia, h = height, center=false);
    translate([+19.0,+24.5,0])cylinder(d = dia, h = height, center=false);
}
module raspberry_bracket() {
    difference() {
        union() {
            cube([85,56,2],center=true);
            raspberry_holes(6, 5);
        }
        raspberry_bracket_holes();
        translate([0,0,-10])raspberry_holes();
    }
}
module raspberry() {
    difference() {
        cube([85,56,17],center=true);
        raspberry_holes();
    }
}
module raspberry_whith_everithing() {
    %raspberry();
    translate([0,0,-14.7])raspberry_bracket();
    translate([0,0,-18.5])raspberry_bracket_support();
    for(A = [1, -1]) {
        translate([-30,A*20,-18.5])
            raspberry_bracket_support();
    }
}
//raspberry_whith_everithing();

raspberry_bracket();