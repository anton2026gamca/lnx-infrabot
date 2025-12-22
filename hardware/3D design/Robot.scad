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

// IR seeker 3D model zo stranky
// %rotate([0,0,180])translate([-21,-21,210])import("mrm-ir-finder3.stl");

module ball() {
    translate([0,0,43/2])sphere(d = 43);
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
module platform_conection_holes(diameter = 3.3) {
    for (A = [247.5: 45:472.5]) {
        rotate([0,0,A])translate([0,100,140]) {
            cylinder(d = diameter, h = 300, center=true);         
        }
    }
}



module middle_wall(USB_hole = 1) {
    translate([0,0,55])for(A = [90, 180, 270]) for(B = [1,-1]) {
        difference() { // screw holes
            union() {
                rotate([0,0,A + 45/2*B])translate([0,-100,20])
                    cylinder(h = 40-0.2, d = 12,center = true);
            }
            
            platform_conection_holes(5.5);
            for(C = [8,31])rotate([0,0,A + 45/2*B])
                translate([0,-95,C]){
                // translate([0,-3,0])cylinder(h=3.1,d=3.3,center=true);
                translate([0,-3,0])cylinder(h=3.1,d=6.3,$fn=6,center=true);
                translate([0,-8,0])cube([6.3,10,3.1],center=true);
            }
        }    
        
        translate([0,0,20])difference() { // wall
            cylinder(h = 40-0.2, d = 215,center=true);
            cylinder(h = 130, d = 210,center=true);
            translate([0,-90,0])
                cylinder(h = 130, d = 180, center=true);
            for(B = [1,-1]) for(C = [8,31])
            rotate([0,0,A + 45/2*B])
                    translate([0,-97.5,C]){
                translate([0,-3,0])cylinder(h=3.1,d=6.3,$fn=6,center=true);
                translate([0,-8,0])cube([6.3,10,3.1],center=true);
            }
            if (USB_hole) {
                translate([55,60,-5])
                    cube([15, 100, 8],center=true);
            }
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
                // translate([0,-3,0])cylinder(h=3.1,d=3.3,center=true);
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
                    // translate([0,-3,0])cylinder(h=3.1,d=3.3,center=true);
                        translate([0,-3,0])cylinder(h=3.1,d=6.3,$fn=6,center=true);
                        translate([0,-8,0])cube([6.3,10,3.1],center=true);
                }
            }
            rotate([0,0,A])translate([0,-100,20])scale([1,1,1])cylinder(h = 40-0.2,d = 90,center = true);
        }
    }
}
module whole_bottom_wall() {
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
module bottom_part () {
    difference() {
        
        // Main part
        translate([0,0,wheel_d/2-15.5]) union() {
            cylinder(d = robot_d, h = 2, center=true);
            translate([0,80,15/2+1])
                cube([60,40,15],center=true);
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
            translate([A*15,80,23])
                cube([26,35, 20], center=true);
  
    }
}

module middle_part (brackets_support = 1) {
    difference() {
        
        // Middle part
        union() {
            translate([0,0,wheel_d/2-15.5 + 42]) 
                cylinder(d = robot_d, h = 2, center=true);
            if (brackets_support) {
                translate([-55,0,57.4])
                    raspberry_bracket_support_all();
                translate([55,15,57.4]) rotate([0,0,90])
                    teensy_board_brackets_holes(6, 5);
            }
        }
        
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
        translate([-55,0,0])        
            raspberry_bracket_holes();
        
        // teensy board
        translate([55,15,76.6]) rotate([0,0,90])
            teensy_board_brackets_holes();
        
        // cable hole
        cylinder(d = 90, h = 200, center=true);
        
        for(A = [1,-1])
            translate([A*15,80,50])cube([26,35, 100], center=true);
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
            translate([0,-70,23.1])kicker();
            kicker_cutout();
            
            bottom_part();
            whole_bottom_wall();
            ball_zone();
            middle_part();
            middle_wall();
            translate([-55,0,65])raspberry_bracket();
        }
    }
}
//all();
middle_part(0);
//middle_wall(0);
//ball_zone();
//kicker_cutout();
//translate([0,-70,23.1])kicker();
//motors();
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
