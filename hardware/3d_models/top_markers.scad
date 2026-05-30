$fn = 90;

text_font_size = 30;
height = 10;

module top_marker(num = 0) {
    num = num % 100;
    translate([0,0,height])
        cylinder(d = 50, h = 2);
    translate([0,0,height + 1])color("green")
        linear_extrude(height = 3)
            text(str(num),
                size = text_font_size,
                font = "JetBrainsMono",
                halign = "center",
                valign = "center"
            );
    difference() {
        cylinder(d = 27.2, h = height, $fn = 12);
        translate([0,0,-5])cylinder(d = 25, h = height + 10);
    }
}
top_marker();