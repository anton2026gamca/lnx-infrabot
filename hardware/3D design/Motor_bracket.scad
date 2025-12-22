wheel_d = 55;
wheel_h = 12.5;
wheel_offset = 90;
robot_d = 215;
robot_h = 220;

// Motor bracket
width = 25;
thickness = 2;
height = 27;
lenght = 52;
num_holes = 7;
hole_set = 6.35;

ballDiameter = 43;
module motor_bracket_holes() {
    for (A = [0:90:359]){
        rotate([0,0,A + 45])
        translate([wheel_offset - 5,0,0])
        for (B = [0:1:num_holes-1]) {
            rotate([0,0,180])
            translate([0,0,-14.5])
            translate([B*hole_set + 6.4,0,0])
            cylinder(d = 3.3, h = 100,center=true);
        }
    }
}
module motor_bracket() {
    rotate([0,0,180])translate([0,0,-14.5])
    difference() {
        union() {
            translate([lenght/2,0,thickness/2])
                cube([lenght,width,thickness],center=true);
            translate([thickness/2,0,height/2])
                cube([thickness,width,height],center=true);
        }
         for (B = [-1, 1]) {
            rotate([0,90,0])translate([B*8.5 - 14.5,0,0])
                cylinder(d = 3.3, h = 100,center=true);
             rotate([90,0,90])translate([B*8.5,14.5,0])
                cylinder(d = 3.3, h = 100,center=true);
        }
        translate([0,0,14.5])rotate([0,90,0])
            cylinder(d = 7.5,h = 100, center=true);
        difference() {
            translate([0,0,27])cube([20,30,25],center=true);
            translate([0,0,14.5])rotate([0,90,0])
                cylinder(d = 25,h = 100, center=true);
        }
        
        
        for (B = [0:1:num_holes-1]) {
            translate([B*hole_set + 6.4,0,0])
                cylinder(d = 3.3, h = 100,center=true);
        }
        for (B = [0:1:num_holes-2]) for (C = [1, -1]){
            translate([B*hole_set + 9.5,C*21.7/2,0]) {
                cylinder(d = 3.3, h = 100,center=true);
                translate([0,C*2,0])cube([3.3,4,100], center=true);
            }
        }
        
    }
}
module motors() {
    translate([0,0,wheel_d/2]) for (A = [0:90:359]){
        rotate([A,90,45]) translate([0,0,wheel_offset-3]) {
            difference() {
                union() {
                    translate([0,0,-(motor_L + 30.8)/2-2.5-10/2])
                      cylinder(h = (motor_L + 30.8), d = 24.4, center=true);
                    translate([0,0,-2.5/2-10/2])
                      cylinder(h = 2.5, d = 7, center=true);
                    translate([0,0,-10/2])
                      cylinder(h = 10, d = 4);
                }
                for(B = [-1, 1]) {
                    translate([0,B*8.5,-10])cylinder(d = 3.3, h = 10);
                }
            }
        }
    }
}


module motor_brackets_all() {
    translate([0,0,wheel_d/2])
    for (A = [0:90:359]){
        rotate([0,0,A + 45])
            translate([wheel_offset - 5,0,0])
                motor_bracket();
    }

}
