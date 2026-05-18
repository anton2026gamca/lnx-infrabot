$fn = 90;
wheel_d = 60;
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

rpi_pos = [0,-30,55];
level_shifter_pos = [-35,32,57.4];

// IR seeker 3D model zo stranky
// %rotate([0,0,180])translate([-21,-21,210])import("mrm-ir-finder3.stl");
module MAXsize() {
    cylinder(d = robot_d, h = robot_h);
}

//
// module uppest_part_new() {
//     translate([0,0,200]) difference() {
//         union() {
//             *translate([0,0,0])rotate([20,0,0]){
//                 translate([0,0,-18]) difference() {
//                     union() rotate([90,0,0]) {
//                         translate([0,0,-5])
//                             cube([25,24,2], center=true);
//                         translate([0,0,-3.12])
//                             camera_holes(4, 3);
//                         // conection to robot construction
//                         
//                     }
//                     rotate([90,0,0])camera_holes();
//                 }
//             }
//             //%translate([0,0,6])rotate([0,0,180])IR_sensor();
//             cube([20,25,2],center=true);
//             translate([0,0,-9])
//                 cube([165, 2.5, 20], center = true);
//             rotate([0,0,180])translate([0,0,3])IR_sensor_holes(4, 6);
//             for(A = [1,-1]) {
//                 translate([A*81.75,0,-9])
//                     cube([12.5,15, 20], center=true);
//             }
//             for (A = [1, -1]) {
//                 translate([A*16.7,-6,-14.5])rotate([0,90,0])
//                     cylinder(d = 9, h = 8,center=true);
//                 translate([A*16.7,-3,-14.5])
//                     cube([8, 6, 9],center=true);
//                 
//             }
//         }
//         translate([0,0,-16])cube([25.4,100,25], center = true);
//         rotate([0,0,180])IR_sensor_holes();
//         translate([0,-6,-14.5])rotate([0,90,0])
//             cylinder(d = 3.3, h = 100,center=true);
//         for(A = [1,-1])
//             translate([A*83,0,0])
//                 cube([11,10.2, 100], center=true);
//         for(A = [1,-1]) for(B = [0: 10:10])
//             translate([A*83,0,B - 13])rotate([90,0,0])
//                 cylinder(d = 3.3, h = 100, center=true);
//         for(A = [1,-1]) for(B = [0:10:10])
//             translate([A*83,25,B - 13])rotate([90,0,0])
//                 cylinder(d = 6.4, h = 50, center=true, $fn = 6);
//         
//          
//         
//     }
// }

module uppest_part_new() {
    height = 20;
    thickness = 2.5;
    ir_rotation_offset = 53;
    ir_height_offset = 10;
    ir_holders_d = 4;
    camera_holder_d = 9;
    camera_holder_offset = 4.5;

    translate([0,0,187]) {
        difference() {
            cube([154, thickness, height], center=true);
            translate([0, 0, -15.5 - height / 2 + camera_holder_d])
                rotate([-45, 0, 0])
                    cube([25.4, 50, 20], center=true);
        }

        translate([0, 0, height / 2 + ir_height_offset]) 
            rotate([0, 0, ir_rotation_offset])
                %IR_sensor();

        for (dir = [1, -1]) {
            rotate([0, 0, ir_rotation_offset]) {
                translate([0, 0, height / 2]) {
                    translate([dir * -7.5, dir * 10, 0]) {
                        difference() {
                            union() {
                                cylinder(d=ir_holders_d, h=ir_height_offset, center=false);
                                translate([0, 0, -4])
                                    cylinder(d1=0, d2=ir_holders_d, h=4, center=false);
                            }

                            translate([0, 0, ir_height_offset])
                                cylinder(d=1.6, h=ir_height_offset, center=true);
                        }
                    }
                }
            }

            translate([dir * 81.75, 0, 0]) {
                difference() {
                    cube([12.5, 15, height], center=true);

                    translate([dir * 1.25, 0, 0]) {
                        cube([11, 10.2, height + 1], center=true);

                        for (hole_dir = [-1, 1]) {
                            translate([0, 0, hole_dir * 5]) rotate([90, 90, 0]) {
                                cylinder(d=3.3, h=100, center=true);
                                cylinder(d=6.4, h=50, center=false, $fn=6);
                            }
                        }
                    }
                }
            }

            translate([dir * 16.7, 0, camera_holder_d / 2 - height / 2]) {
                rotate([0,90,0]) {
                    translate([0, -camera_holder_offset - thickness / 2, 0]) {
                        difference() {
                            union() {
                                cylinder(d=camera_holder_d, h=8, center=true);
                                translate([0, camera_holder_offset / 2, 0])
                                    cube([camera_holder_d, camera_holder_offset, 8], center=true);
                            }
                            cylinder(d=3.3, h=16, center=true);
                        }
                    }
                }
            }
        }
    }
}

module uppest_part() {
    translate([0,0,200]) difference() {
        union() {
            *translate([0,0,0])rotate([20,0,0]){
                translate([0,0,-18]) difference() {
                    union() rotate([90,0,0]) {
                        translate([0,0,-5])
                            cube([25,24,2], center=true);
                        translate([0,0,-3.12])
                            camera_holes(4, 3);
                        // conection to robot construction
                        
                    }
                    rotate([90,0,0])camera_holes();
                }
            }
            scale([1.3,1,1])cylinder(d = 35, h = 4, center = true);
            for(A = [1,-1]) {
                translate([A*46,0,-8])
                    cube([60, 2.5, 20], center = true);
                translate([A*81.75,0,-8])
                    cube([12.5,15, 20], center=true);
            }
        }
        IR_sensor_bracket_holes();
        for(A = [1,-1])
            translate([A*83,0,0])
                cube([11,10.2, 100], center=true);
        for(A = [1,-1]) for(B = [0: 10:10])
            translate([A*83,0,B - 13])rotate([90,0,0])
                cylinder(d = 3.3, h = 100, center=true);
        for(A = [1,-1]) for(B = [0:10:10])
            translate([A*83,25,B - 13])rotate([90,0,0])
                cylinder(d = 6.4, h = 50, center=true, $fn = 6);
        
         
        
    }
}
module handle() {
    translate([0,0,0])difference() {
        union() {
            for(A = [1, -1]) {
                translate([A*83,0,170 + 7.5])cube([10,10,80 + 15], center=true);
                translate([A*60,0,231 + 15])rotate([0,A*-45,0])
                    cube([10,10,68], center=true);
            }
            translate([0,0,253.5 + 15])cube([80,10,10], center=true);
        }
        for(A = [1,-1]) {
            for (B = [10, 0]) 
                translate([A*83,0,B+135])rotate([90,0,0])
                    cylinder(d = 3.3, h = 200, center=true);
        }
        for(A = [1,-1]) for(B = [0:5:60])
            translate([A*83,0,B + 157])rotate([90,0,0])
                cylinder(d = 3.3, h = 100, center=true);
    }
}
module handle_bracket() {
    for (A = [0, 180]) rotate([0,0,A]) {
        difference() {
            union() {
                translate([83,0,99])cube([12,85,3], center=true);
                translate([83,0,105])cube([12,60.81,11.33], center=true);
                for (A = [1, -1]) translate([0,0,134]) rotate([45*A,0,0]){
                    translate([83,A*-17,0])cube([12,42,10], center=true);
                }
                translate([83,0,135])cube([12,15,30], center=true);
            }
            platform_conection_holes();
            for(C = [1,-1]) {
                translate([C*72,0,70])
                    cube([26,35, 100], center=true);
                translate([C*83,0,110])
                    cube([11,10.2, 100], center=true);
                for (B = [10, 0]) 
                    translate([C*83,0,B+135])rotate([90,0,0])
                        cylinder(d = 3.3, h = 200, center=true);
            }
            if (A == 0)
                for (B = [10, 0]) 
                    rotate([90,0,0])translate([83,B+135,-20])
                        cylinder(d = 6.4, h = 50, center=true, $fn = 6);
            else
                for (B = [10, 0]) 
                    rotate([90,0,0])translate([83,B+135,20])
                        cylinder(d = 6.4, h = 50, center=true, $fn = 6);

            
        }
    }
    
}



module LED_holes(diodeDia=7,height=10){
    
            for(A=[0:15:359])rotate([0,0,A])translate([0,45,0])cylinder(d=diodeDia,h=height,center=true);
             if(0){
                 rotate([0,0,0])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,30])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,60])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,90])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,120])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,150])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,180])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,210])translate([0,66,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,238.7])translate([0,65.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,270])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,300])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0,330])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
             }
             if(0){
                 rotate([0,0, 0+16.25])translate([0,66,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 30+16.7])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 60+14.25])translate([0,66,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 90+15.6])translate([0,66,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 120+15])translate([0,66.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 150+14.3])translate([0,66.2,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 180+13.7])translate([0,66,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 210+13.3])translate([0,65.5,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 239+14.58])translate([0,65.8,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 270+14.4])translate([0,65.7,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 300+15.2])translate([0,65.7,0])cylinder(d=diodeDia,h=height,center=true);
                 rotate([0,0, 330+15.8])translate([0,66.3,0])cylinder(d=diodeDia,h=height,center=true);
             }


}
module LED_shield() {
    difference() {
        union() {
            translate([0,0,-5])cylinder(d=100,h=5);
            for (A = [1, -1]) {
                translate([A*50,0,-1])cube([20, 10, 2],center=true);
            }
        }
        translate([0,0,-6])cylinder(d=95,h=7);
        for (A = [1, -1]) {
            translate([A*55,0,-1])cylinder(d=3.3, h=100,center=true);
        }
    }
}
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
    difference() {
        union() {
            translate([0,0,55])for(A = [90, 180, 270]) for(B = [1,-1]) {
                difference() { // screw holes
                    translate([0,0,20])
                        platform_conection_holes(13, 40-0.2);
                    platform_conection_holes(7);
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
            //back camera moved to center of robot
            *intersection() {
                translate([0,110,95])scale([1.7,1,1])
                    sphere(d = 51);
                translate([0,84,44.9])cube([100,30,100],center = true);
            }
        }
        //back camera moved to center of robot
        *translate([0,110,95])scale([1.7,1,1])
            sphere(d = 50);
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
            translate([0,0,55/2-15.5]) union() {
                cylinder(d = robot_d, h = 2, center=true);
                for(A = [1,-1])
                translate([A*72,0,10])
                    cube([30,39, 20], center=true);
                
                //*cylinder(d = 105, h = 3);
                difference(){
                    translate([0,0,-4])cylinder(d=robot_d,h=4);
                    //translate([0,0,-6])cylinder(d=95,h=6);
                }
                
                
            }
            // Line sensor
            translate([0,0,10])LED_holes();
            difference(){
                translate([0,0,0])cylinder(d = 80, h = 200, center=true);
                        
                translate([0,0,7])difference() {
                    for(A=[1,-1]) {
                        translate([A*42,0,1.5])cube([20,15,3],center=true);                        
                        translate([0,A*35,1.5])cube([15,20,3],center=true);
                    }
                    for(A=[1,-1]) {
                        translate([A*37,0,0])cylinder(d=3.3,h=100,center=true);
                        translate([0,A*30,0])cylinder(d=3.3,h=100,center=true);
                    }
                }
                
            }
            translate([0,0,14.5])cylinder(d = 100, h = 10, center=true);
            
            // Connection to upper parts
            platform_conection_holes();
            
            // LED shield holes
            for (A = [1, -1]) {
                translate([A*55,0,-1])cylinder(d=3.3, h=100,center=true);
            }
            
            // Wheels
            for (angle = [45, 135, 225, 315]) {
                rotate([0, 0, angle]) {
                    side = 45;
                    d_outer = side / sin(180 / 8);
                    d_inner = side / tan(180 / 8);
                    translate([wheel_offset + d_inner / 2 - 5, 0, 0]) {
                        rotate([0, 0, 360 / 16])
                            cylinder(wheel_h + 10, d=d_outer, $fn=8);
                    }
                } 
            }
            wheels_cutout();

            %wheels();
            
            
            // Motor brackets holes
            motor_bracket_holes();
            
            // Driver holes
            motor_driver_bracket_holes();
            
            // Ball zone holes
            ball_zone_holes();
            
            // Kicker holes
            translate([0,-70,23.1])kicker_holes();
            
            // Batery
            for(A = [1,-1])
                translate([A*72,0,25])
                    cube([26,35, 25], center=true);
      
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
                translate([0,0,55/2-15.5 + 42]) 
                    cylinder(d = robot_d, h = 2, center=true);
                if (brackets_support) {
                    translate(rpi_pos) rotate([0,0,0])
                        raspberry_holes(6, 5);
                    *translate([30,-12.5,57.4]) rotate([0,0,90])
                        teensy_board_brackets_holes(6, 5);
                    translate(level_shifter_pos)
                        level_shifter_holes(6, 5);
                }
            }
            // raspberry
            translate(rpi_pos) rotate([0,0,0])
                translate([0,0,-10])raspberry_holes();
            
            translate(level_shifter_pos)
                level_shifter_holes();
            
            // back side
            translate([0,70,0])cube([50,35,200],center=true);
            
            // Connection to other parts
            platform_conection_holes();
            
            // Motor brackets holes - if mounting it from top
            motor_driver_bracket_holes();
            
            // Motor brackets holes
            //translate([0,0,100])motor_bracket_holes();
            
            // Ball zone holes
            ball_zone_holes();
            
            // Kolesa
            wheels_cutout();
            
            // ball zona
            translate([0,0,40])ball_zone_cutout();
            
            
            
            
            // level_shifter
            translate([-40,37,70])
                level_shifter_holes();
            
            // cable hole
            cylinder(d = 30, h = 200, center=true);
            
            // batery
            for(A = [1,-1])
                translate([A*72,0,23])
                    cube([26,35, 100], center=true);
        }
        
    }
}



module upper_part () {
    difference() {
        translate([0,0,96]) union() {
            
            intersection() {
                cylinder(d=robot_d,h=2,center=true);
                            
                translate([0,12,0])rotate([0,0,0])
                    cube([180,180,200],center=true);
            
            
            }
            translate([0,100,19])difference(){
                //back camera moved to center of robot
                *for (A = [1, -1]) {
                    translate([A*16.7,-25.5,0])rotate([0,90,0])
                        cylinder(d = 9, h = 8,center=true);
                    translate([A*16.7,-25.5,-10])
                        cube([8,9,20],center=true);

                }
                //back camera moved to center of robot
                *translate([0,-25.5,0])rotate([0,90,0])
                    cylinder(d = 3.3, h = 80,center=true);
            }
            
            
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
        // Back camera hole
        /* //back camera moved to center of robot
        translate([0,100,95])cube([25.4,60,50], center=true);
        translate([0,110,95])scale([1.7,1,1])
            sphere(d = 50);
        */
        // BUTTON HOLE
        
        
        // platform conection
        platform_conection_holes();
        // IR sensor
        *IR_sensor_bracket_holes();
        // front
        *translate([0,-89.2,100])
            cube([200,50,100],center=true);
        translate([0,-120,100]) scale([1.05,1,1])
            cylinder(h = 100, d = 200,center=true);
        
        
        // batery
        for(A = [1,-1])
            translate([A*72,0,50])
                cube([26,35, 100], center=true);
        
        // SOCCER COMUNICATION MODUL 
                    //translate([-73-22.86/2,-34,100])
        /*translate([-22.86/2,40,100]) {
            cube([2.54+0.5,6*2.54+0.5,100], center = true);
            translate([22.86,2.54,0])
                cube([2.54+0.5,4*2.54+0.5,100],center=true);
        }*/
        // teensy buttons
        translate([-22.86/2,15,100])
            cube([2.54+0.5,6*2.54+0.5,100], center = true);
        // LCD display
        translate([38/2,40,100])
            cube([2.54+0.5,4*2.54+0.5,100], center = true);
        // rpi buttons
        translate([0,53,100])
                cube([(4*3+1)*2.54+0.5,2.54+0.5,100], center = true);
        
        // BNO Compass
        translate([50+8*2.54/2,50,100])rotate([0,0,45]) {
            cube([2.54+0.5,6*2.54+0.5,200], center = true);
            translate([7 * 2.54,0,0])
                cube([2.54+0.5,4*2.54+0.5,200],center=true);
        }
        
        // Switch ON / OFF
        translate([-55,65,50])rotate([0,0,45])cube([19,13,100], center=true);
        
        
    }

    translate([0,0,96])difference() {
        for (A = [1, -1]) {
            translate([A*(16.7-1),-24.5,3.5]) {
                rotate([0,90,0])cylinder(d = 9, h = 8-2,center=true);
                translate([0,5,-3.5])cube([8-2, 10, 2], center=true);
            }
            
        }
        translate([0,-24.5,3.5])rotate([0,90,0])
            cylinder(d = 3.3, h = 80,center=true);
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
//handle_bracket();
//handle();
//motor_brackets_all(180);
//translate([0,0,47.5])motor_brackets_holders();
//all();
//IR_sensor_bracket();
//ball();
*bottom_part();
*middle_part();
upper_part();
*uppest_part();
*uppest_part_new();
//bottom_part();
*bottom_wall();
*middle_wall(0);

//wheels_cutout();
*ball_zone();
*translate([0,99,115])rotate([0,0,180]){//front camera
    camera_bracket_holder();
    camera_bracket();
    camera();
}
*translate([0,50,190])rotate([0,0,180]){//back camera
    rotate([0,180,0])translate([0,0,-3.5])camera_bracket_holder_new();
    translate([0,42.5,-21.5])rotate([-20,180,180]){
        camera_bracket();
        camera();
    }
}
*ball_zone();
//kicker_cutout();
//translate([0,-70,23.1])kicker();
//motors();

/*
middle_part(0);

%translate([-25,-20,60.4])rotate([0,0,90])
    raspberry_bracket();
%translate([30,-12.5,76.6]) rotate([0,0,90])
    teensy_board_bracket();
#translate([-40,37,70]) level_shifter();

%bottom_wall();
/**/

//bottom_part();






*color("red", 0.5)translate([0,-40,75])rotate([0,0,0]) {
    raspberry();
    translate([0,0,-14.6])raspberry_bracket();
}

*translate([0,-robot_d/2+7.5 + 10,0])
    cube([100,15,100], center=true);
*translate([0,-30,75])rotate([0,0,0])
    translate([0,0,-14.6])raspberry_bracket();

*translate([55,15,76.6]) rotate([0,0,90])
    teensy_board_bracket();
*translate([0,0,10])translate(level_shifter_pos)level_shifter();
//  diera na baterku - 35x26
*for(A = [1,-1])
    #translate([A*15,80,50])cube([26,35, 100], center=true);
// translate([0,0,70])cube([100,50,2], center=true);
*for (A = [1,-1])
    translate([A*75,0,20])cube([35, 60, 2], center=true);

