$fn = 90;
wheel_d = 55;
wheel_h = 12.5;
wheel_offset = 90;
robot_d = 215;
robot_h = 220;

ballDiameter = 43;


use<Driver_bracket.scad>;

//IR seeker 3D model zo stranky
//%rotate([0,0,180])translate([-21,-21,210])import("mrm-ir-finder3.stl");


//Motor 
motor_L = 17;

//Motor bracket
width = 25;
thickness = 2;
height = 27;
lenght = 52;
num_holes = 7;
hole_set = 6.35;




module IR_seeker() {
    translate([0,0,200])difference() {
        union() {
            cylinder(h = 2, d = 40, center=true);
            for (A = [0:30:359]) {
                rotate([0,0,A])translate([0,20,3])cube([5,5,6], center=true);
            }
            for (A = [0:60:359]) {
                rotate([0,0,A])translate([0,20,-5])cube([5,5,10], center=true);
            }
        }
            // d = 1.95 -> 2.3
        translate([7.5,-10,0])cylinder(h = 100, d = 2.3, center=true);
        translate([-7.5,-10,0])cylinder(h = 100, d = 2.3, center=true);
        translate([-7.5,10,0])cylinder(h = 100, d = 2.3, center=true);
        translate([0,-10,0])cube([10,10,100], center=true);
        
    }
}





module ball_zone_holes(diameter = 3.3, height = 100) {
    for (A = [1, -1]) {
        translate([A*36,-73,0])
            cylinder(d = diameter, h = height, center=true);
    }
}
module ball_zone() {
    translate([0,0,33]) difference() {
        union() {
            translate([0,-90,0]) cube([100,30,40-0.2],center=true);
            ball_zone_holes(12, 40-0.2);
        }
        ball_zone_cutout();
        for (A=[45:90:360]) {
            rotate([0,0,A])translate([0,84,0])cube([65,4,80],center=true);
        }
        translate([0,0,-33])wheels_cutout();
        ball_zone_holes(5.6);
        
    }
}
module wall_behind_wheel() {
    difference() {
        cube([2,70,40-0.2],center=true);
        translate([0,0,-5])
            rotate([0,90,0])cylinder(d = 27, h = 200, center=true);
        translate([0,0,-18.5])cube([10,27,30],center=true);
    }       
}
module wall_between_wheels() { 
    translate([0,0,13])for(A = [90, 180, 270]) for(B = [1,-1]) {
        difference() {
            union() {
                rotate([0,0,A + 45/2*B])translate([0,-100,20])
                    cylinder(h = 40-0.2, d = 12,center = true);
            }
            platform_conection_holes(5.5);
            for(C = [8,31])rotate([0,0,A + 45/2*B])
                translate([0,-95,C]){
                //translate([0,-3,0])cylinder(h=3.1,d=3.3,center=true);
                translate([0,-3,0])cylinder(h=3.1,d=6.3,$fn=6,center=true);
                translate([0,-8,0])cube([6.3,10,3.1],center=true);
            }
        }    
        intersection() {
            difference() {
                cylinder(h = 100, d = 215,center=true);
                cylinder(h = 130, d = 210,center=true);
                for(B = [1,-1]) for(C = [8,31]) rotate([0,0,A + 45/2*B])
                    translate([0,-97.5,C]){
                    //translate([0,-3,0])cylinder(h=3.1,d=3.3,center=true);
                        translate([0,-3,0])cylinder(h=3.1,d=6.3,$fn=6,center=true);
                        translate([0,-8,0])cube([6.3,10,3.1],center=true);
                }
            }
            rotate([0,0,A])translate([0,-100,20])scale([1,1,1])cylinder(h = 40-0.2,d = 90,center = true);
        }
    }
}
module whole_wall() {
    wall_between_wheels();
    translate([0,0,33]) for (A = [45: 90: 360]) difference() {
        rotate([0,0,A]) {
            translate([88,0,0])for (B = [1, -1]) {
                translate([0,B*34,0])cube([10,2,40-0.2],center=true);
            }
            translate([84,0,0])wall_behind_wheel();
            
        }
        translate([0,-115,0])rotate([0,0,45])cube([100,100,100],center=true);
    }
}
module platform_conection_holes(diameter = 3.3) {
    for (A = [247.5: 45:472.5]) {
        rotate([0,0,A])translate([0,100,140]) {
            cylinder(d = diameter, h = 300, center=true);         
        }
    }
}
module motor_driver_brackets_all() {
    for(A = [90, 270]) {
        rotate([0,0,A])translate([0,75,14+25])driver_holder();
    }
}
module motor_driver_bracket_holes() {
    for(A = [1, -1]) {
        translate([A*92,0,100])
            cylinder(d = 3.3, h = 200, center=true);
        for(B = [1, -1]) {
            translate([A*65,B*30,100])
                cylinder(d = 3.3, h = 200, center=true);
        }
    }
}
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





module MAXsize() {
    cylinder(d = robot_d, h = robot_h);
}

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
module wheel_conector() {
    rotate([0,0,45])translate([0,0,wheel_d/2]) for (A = [0:90:359]){
        rotate([A,90,0]) translate([0,0,wheel_offset-5/2]) {
            difference() {
                cylinder(h = 5, d = 17.5, center = true);
                cylinder(h = 10, d = 3.5, center=true);
            }
        }
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
module down_part () {
    difference() {
        
        //Main part
        translate([0,0,wheel_d/2-15.5])
            cylinder(d = robot_d, h = 2, center=true);
        
        //Connection to upper parts
        platform_conection_holes();
        
       
        //Kolesa
        wheels_cutout();
        //Motor brackets holes
        motor_bracket_holes();
        
        //Driver holes
        motor_driver_bracket_holes();
          
        //Ball zone
        ball_zone_cutout();
        
        //Ball zone holes
        ball_zone_holes();
  
    }
}

module middle_part () {
    difference() {
        
        //Middle part
        translate([0,0,wheel_d/2-15.5 + 42])
            cylinder(d = robot_d, h = 2, center=true);
        
        //Connection to other parts
        platform_conection_holes();
        
        //Motor brackets holes - if mounting it from top
        motor_driver_bracket_holes();
        
        
        
        //Ball zone holes
        ball_zone_holes();
        
        //Kolesa
        wheels_cutout();
        
        //ball zona
        translate([0,0,40])ball_zone_cutout();
        
        //cable hole
        cylinder(d = 50, h = 200, center=true);
        
        
    }
}



*intersection() {
    color("white")MAXsize();
    union() {
        #wheels();
        down_part();
        //color("blue", 0.3)motor_brackets_all();
        //color("lightgray", 0.3)motors();
        //wheel_conector();
        //motor_driver_brackets_all();
        middle_part();
        whole_wall();
        ball_zone();
    }
}

//motor_brackets_all();
ball_zone();
//IR_seeker();
*translate([0,-robot_d/2+7.5 + 10,0])
    cube([100,15,100], center=true);

// diera na baterku - 35x26

*for (A = [1,-1])
    translate([A*75,0,20])cube([35, 60, 2], center=true);
