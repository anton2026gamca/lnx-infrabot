$fn = 90;
wheel_d = 55;
wheel_h = 12.5;
wheel_offset = 90;
robot_d = 215;
robot_h = 220;

ballDiameter = 43;

use <Wheels.scad>
use <Driver_bracket.scad>;
use <Raspberry_bracket.scad>;
use <Motor_bracket.scad>;
use <Teensy_board_bracket.scad>;
use <Kicker.scad>;
use <Level_shifter.scad>;
use <Camera_bracket.scad>;
use <IR_sensor_bracket.scad>;

// IR seeker 3D model zo stranky
// %rotate([0,0,180])translate([-21,-21,210])import("mrm-ir-finder3.stl");

module ball() {
    translate([0,-104.5,43/2])sphere(d = 43);
}


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
            //  d = 1.95 -> 2.3
        translate([7.5,-10,0])cylinder(h = 100, d = 2.3, center=true);
        translate([-7.5,-10,0])cylinder(h = 100, d = 2.3, center=true);
        translate([-7.5,10,0])cylinder(h = 100, d = 2.3, center=true);
        translate([0,-10,0])cube([10,10,100], center=true);
        
    }
}

//
module MAXsize() {
    cylinder(d = robot_d, h = robot_h);
}
//
module platform_conection_holes(diameter = 3.3, height = 500) {
    for (A = [247.5: 45:472.5]) {
        if (A > 320 && A < 400)
            rotate([0,0,A])translate([0,100,00]) {
                cylinder(d = diameter, h = height, center=true);      
        }
        else
            rotate([0,0,A])translate([0,90,0]) 
                cylinder(d = diameter, h = height, center=true);
    }
}



module middle_wall(USB_hole = 1) {
    translate([0,0,55])for(A = [90, 180, 270]) for(B = [1,-1]) {
        difference() { // screw holes
            translate([0,0,20])
                platform_conection_holes(13, 40-0.2);
            platform_conection_holes(6.2);
        }
        for (A = [1, -1]) {
            translate([A*89,0,20])cube([2,70,40-0.2],center=true);
        }
        translate([0,98,20])cube([80,2,40-0.2],center=true);
        
        translate([0,30,20])difference() { // wall
            scale([1,0.855,1])cylinder(h = 40-0.2, d = 180,center=true);
            scale([1,0.855,1])cylinder(h = 130, d = 175,center=true);
            translate([0,-50,0])
                cube([200,100,100],center=true);
                cube([80,300,100],center=true);
            
            
        }
        
    
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

module bottom_wall() {
     intersection() {
         *translate([0,6,0])rotate([0,0,0])
            cube([180,180,200],center=true);
         union () {
            for (B = [1, -1])difference() {
                /*union() {
                    rotate([0,0,180 + 45/2*B])translate([0,-100,33])
                        cylinder(h = 40-0.2, d = 12,center = true);
                }*/
                translate([0,0,33])
                    platform_conection_holes(12, 40-0.2);
                platform_conection_holes(6.2);

                
            }
            
            for (A = [1, -1]) {
                translate([A*89,0,33])
                    cube([2, 82, 40-0.2], center=true);
            }
            
            translate([0,0,33]) for (A = [45: 90: 360]) difference() {
                rotate([0,0,A]) {
                    translate([88,0,0])for (B = [1, -1]) {
                        translate([0,B*34,0])
                            cube([8,2,40-0.2],center=true);
                    }
                    translate([84,0,0])wall_behind_wheel();
                    
                }
                translate([0,-115,0])rotate([0,0,45])
                    cube([100,100,100],center=true);
                platform_conection_holes(6.2);
            }
        }
    }
}
module bottom_wall_back_part() {
    %difference() {
        union() {
            rotate([0,0,180 + 45/2*B])translate([0,-100,33])
                cylinder(h = 40-0.2, d = 12,center = true);
        }
        platform_conection_holes(6.2);
    }
    
}
module bottom_part () {
    intersection() {
        translate([0,12,0])rotate([0,0,0])
            cube([180,180,200],center=true);
        difference() {
            
            // Main part
            translate([0,0,wheel_d/2-15.5]) union() {
                cylinder(d = robot_d, h = 2, center=true);
                for(A = [1,-1])
                translate([A*72,0,10])
                    cube([30,39, 20], center=true);
            }
            // Connection to upper parts
            platform_conection_holes();
            
           
            // Kolesa
            wheels_cutout();
            
            // Motor brackets holes
            motor_bracket_holes();
            
            // Driver holes
            motor_driver_bracket_holes();
              
            // Ball zone
            ball_zone_cutout();
            
            // Ball zone holes
            ball_zone_holes();
            
            // Kicker holes
            translate([0,-70,23.1])kicker_holes();
            
            // Batery
            for(A = [1,-1])
                translate([A*72,0,23])
                    cube([26,35, 20], center=true);
      
        }
    }
}

module middle_part (brackets_support = 1) {
    intersection() {
        translate([0,12,0])rotate([0,0,0])
            cube([180,180,200],center=true);
        difference() {
            
            // Middle part
            union() {
                translate([0,0,wheel_d/2-15.5 + 42]) 
                    cylinder(d = robot_d, h = 2, center=true);
                if (brackets_support) {
                    translate([-25,-20,57.4]) rotate([0,0,90]) 
                        raspberry_bracket_support_all();
                    translate([30,-12.5,57.4]) rotate([0,0,90])
                        teensy_board_brackets_holes(6, 5);
                    translate([-50,39,57.4])
                        level_shifter_holes(6, 5);
                }
            }
            
            // back side
            translate([0,70,0])cube([50,35,200],center=true);
            
            // Connection to other parts
            platform_conection_holes();
            
            // Motor brackets holes - if mounting it from top
            motor_driver_bracket_holes();
            
            
            
            // Ball zone holes
            ball_zone_holes();
            
            // Kolesa
            wheels_cutout();
            
            // ball zona
            translate([0,0,40])ball_zone_cutout();
            
            // raspberry
            translate([-25,-20,0]) rotate([0,0,90])        
                raspberry_bracket_holes();
            
            // teensy board
            translate([30,-12.5,76.6]) rotate([0,0,90])
                teensy_board_brackets_holes();
            
            // level_shifter
            translate([-50,39,0])
                level_shifter_holes();
            
            // cable hole
            //cylinder(d = 90, h = 200, center=true);
            
            // batery
            for(A = [1,-1])
                translate([A*72,0,23])
                    cube([26,35, 100], center=true);
        }
    }
}



module upper_part () {
    intersection() {
        translate([0,12,0])rotate([0,0,0])
            cube([180,180,200],center=true);
        difference() {
            translate([0,0,96]) union() {
                cylinder(d=robot_d,h=2,center=true);
                /*// SOCCER COMUNICATION MODUL
                translate([-30-22.86/2,0,10]) {
                    cube([2.54+4,6*2.54+4,20], center = true);
                    translate([22.86,2.54,0])
                        cube([2.54+4,4*2.54+4,20],center=true);
                }
                // BNO Compass
                translate([40-7*2.54/2,0,10])rotate([0,0,45]) {
                    cube([2.54+4,6*2.54+4,20], center = true);
                    translate([7 * 2.54,0,0])
                        cube([2.54+4,4*2.54+4,20],center=true);
                }*/
            }
            // BUTTON HOLE
            
            // Motor brackets holes - if mounting it from top
            motor_driver_bracket_holes();
            // platform conection
            platform_conection_holes();
            // IR sensor
            IR_sensor_bracket_holes();
            // front
            *translate([0,-89.2,100])
                cube([200,50,100],center=true);
            translate([0,-120,100]) scale([1.05,1,1])
                cylinder(h = 100, d = 200,center=true);
            // cable hole
            translate([30,0,0])
                cylinder(d = 30, h = 300,center=true);
            // camera cable hole
            translate([-30,0,100]) cube([5,20,100], center=true);
            // batery
            // batery
            for(A = [1,-1])
                translate([A*72,0,50])
                    cube([26,35, 100], center=true);
            
            // SOCCER COMUNICATION MODUL 
                        //translate([-73-22.86/2,-34,100])
            translate([-22.86/2,60,100]) {
                cube([2.54+0.5,6*2.54+0.5,100], center = true);
                translate([22.86,2.54,0])
                    cube([2.54+0.5,4*2.54+0.5,100],center=true);
            }
            // BNO Compass
            translate([50+8*2.54/2,50,100])rotate([0,0,45]) {
                cube([2.54+0.5,6*2.54+0.5,200], center = true);
                translate([7 * 2.54,0,0])
                    cube([2.54+0.5,4*2.54+0.5,200],center=true);
            }
            
            // Button ON / OFF
            translate([-55,65,50])cube([21,15,100], center=true);
            
            
        }
    }
}
module all(){
    intersection() {
        MAXsize();
        union() {
            *#wheels();
            //color("blue", 0.3)motor_brackets_all();
            //color("lightgray", 0.3)motors();
            //wheel_conector();
            //motor_driver_brackets_all();
            //translate([0,-70,23.1])kicker();
            kicker_cutout();
            
            bottom_part();
            bottom_wall();
            bottom_wall_back_part()
            ball_zone();
            middle_part();
            middle_wall();
            upper_part();
            
            %translate([-55,3,65])raspberry_bracket();
            %translate([55,15,76.6]) rotate([0,0,90])
                teensy_board_bracket();
            
            #translate([0,0,96 + 4 + 80]) {
                IR_sensor_bracket();
                //IR_sensor();
            }
            %translate([-58,50,70]) level_shifter();
        }
    }
}


//all();
//IR_sensor_bracket();
//ball();
*middle_part();
*middle_wall();
upper_part();
//bottom_part();
//bottom_wall();

//ball_zone();
//kicker_cutout();
//translate([0,-70,23.1])kicker();
//motors();

/*
middle_part(0);
//middle_wall(0);

%translate([-25,-20,60.4])rotate([0,0,90])
    raspberry_bracket();
%translate([30,-12.5,76.6]) rotate([0,0,90])
    teensy_board_bracket();
#translate([-50,39,70]) level_shifter();

%bottom_wall();
/**/

//bottom_part();






*color("red", 0.5)translate([-55,0,75])rotate([0,0,0]) {
    raspberry();
    translate([0,0,-14.6])raspberry_bracket();
}

*translate([0,-robot_d/2+7.5 + 10,0])
    cube([100,15,100], center=true);
*%translate([-55,0,75])rotate([0,0,0])
    translate([0,0,-14.6])raspberry_bracket();

*%translate([55,15,76.6]) rotate([0,0,90])
    teensy_board_bracket();

//  diera na baterku - 35x26
*for(A = [1,-1])
    #translate([A*15,80,50])cube([26,35, 100], center=true);
// translate([0,0,70])cube([100,50,2], center=true);
*for (A = [1,-1])
    translate([A*75,0,20])cube([35, 60, 2], center=true);
