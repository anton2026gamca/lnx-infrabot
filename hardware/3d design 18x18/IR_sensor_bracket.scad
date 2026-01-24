$fn = 90;



module IR_sensor_bracket_holes(dia = 3.3, height = 200) {
    for (A = [0:90:359]) {
        rotate([0,0,A])translate([8,0,0])
            cylinder(d = dia, h = height, center=true);
    }
}
module IR_sensor_holes(dia = 2, height = 200) {
    translate([-7.5,10,0])
        cylinder(d = dia, h = height, center=true);
    translate([7.5,10,0])
        cylinder(d = dia, h = height, center=true);
    translate([7.5,-10,0])
        cylinder(d = dia, h = height, center=true);
}
module IR_sensor_bracket() {
    difference() {
        union() {
            translate([0,0,-11])
                cylinder(d = 42, h = 2, center=true);
            for (A = [0:60:359]) {
                rotate([0,0,A]) translate([0,15.5,-6]) difference() {
                    cylinder(h = 10, d = 8,center=true);
                    cylinder(h = 100, d = 7,center=true);
                    translate([0,5,0])
                        cube([3,10,100], center=true);
                }
            }
        }
        // cable hole
        cylinder(d = 10, h = 100, center=true);
        
        IR_sensor_bracket_holes();
        //IR_sensor();
        IR_sensor_holes();
    }
}
module IR_sensor() {
    translate([-21,-21,0])
        import("mrm-ir-finder3.stl");
}
IR_sensor_bracket();
%IR_sensor();
//translate([0,0,-5])cube(10, center=true);

