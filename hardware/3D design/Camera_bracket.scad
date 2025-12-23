$fn = 90;

use <Robot.scad>;
use <IR_sensor_bracket.scad>;

camera_pos_x = 0;
camera_pos_y = 0;
camera_pos_z = 0;

camera_angle_x = 0;
camera_angle_y = 0;
camera_angle_z = 0;
module camera_bracket_holder() {
    translate([0,50,0]) intersection() {
        difference() {
            union () {
                translate([0,-5,0])
                    cube([41.4,34,2],center=true);
                for (A = [1, -1]) {
                    translate([A*16.7,-25.5,0])rotate([0,90,0])
                        cylinder(d = 9, h = 8,center=true);
                    
                }
            }
            translate([0,0,0])IR_sensor_bracket_holes();
            
            translate([0,-29,0])cube([25.4,20,100],center=true);
            translate([0,-25.5,0])rotate([0,90,0])
                cylinder(d = 3.3, h = 80,center=true);
            
        }
        translate([0,-35,0]) scale([1,2.1,1])
            cylinder(d = 44, h = 100, center=true);
    }
}
module camera_holes(dia = 2.3, height = 200) {
    translate([-10.5,10,0])
        for (A = [21, 0]) for (B = [-12.5, 0]) {
        translate([A,B,0])
            cylinder(d = dia, h = height, center=true);
    }
}
module camera_bracket() {
    translate([camera_pos_x,camera_pos_y,camera_pos_z])
    rotate([camera_angle_x,camera_angle_y,camera_angle_z])
    translate([0,15.5,-5]) difference() {
        union() rotate([90,0,0]) {
            translate([0,0,-5])
                cube([25,24,2], center=true);
            translate([0,0,-3.12])
                camera_holes(4, 3);
            // conection to robot construction
            for (A = [1, -1]) for (H = [5, -9]) {
                translate([A*8.5,H,-9])rotate([0,90,0])
                    cylinder(d = 9, h = 8,center=true);
                
            }
        }
        for (H = [5, -9]) translate([0,9,H])rotate([0,90,0])
            cylinder(d = 3.3, h = 80,center=true);
        rotate([90,0,0])camera_holes();
    }
}
module camera() {
    translate([camera_pos_x,camera_pos_y,camera_pos_z])
    rotate([camera_angle_x,camera_angle_y,camera_angle_z])
        translate([-10.5,15.5,10 - 5])rotate([90,0,0])
            import("B0310.stl", center=true);
}



//camera_bracket_holder();
camera_bracket();
%camera();

