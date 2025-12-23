$fn = 90;

module camera_holes(dia = 2.3, height = 200) {
    translate([-10.5,10,0])
        for (A = [21, 0]) for (B = [-12.5, 0]) {
        translate([A,B,0])
            cylinder(d = dia, h = height, center=true);
    }
}
module camera_bracket() {
    difference() {
        union() {
            translate([0,0,-5])
                cube([25,24,2], center=true);
            translate([0,0,-3.12])
                camera_holes(4, 3);
        }
        camera_holes();
    }
}
module camera() {
    translate([-10.5,10,0])import("B0310.stl", center=true);
}
%camera_bracket();
camera();

