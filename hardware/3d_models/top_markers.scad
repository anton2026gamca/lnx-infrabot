$fn = 90;

text_font_size = 35;
height = 20;

module top_marker(mark = "0") {
    text_size = (len(mark) == 1) ? text_font_size :
                (len(mark) == 2) ? text_font_size * 2/3 :
                0;//invalid
    difference() {
        translate([0,0,height])
            cylinder(d = 50, h = 2);
        translate([0,0,height + 1])
            linear_extrude(height = 3)
                text(str(mark),
                    size = text_size,
                    font = "JetBrainsMono",
                    halign = "center",
                    valign = "center"
                );
    }
    difference() {
        cylinder(d = 27.2, h = height, $fn = 12);
        translate([0,0,-5])cylinder(d = 25, h = height + 10);
    }
}
module all_top_markers() {
    parameters = ["1", "2", "3", "4", "A1", "A2", "B1", "B2"];
    for (A = [0:1:7]) {
        translate([55 * (A % 4 - 1.5),55* (sign(A-0.5 - 3))/2,0])
            top_marker(parameters[A]);
    }
}
*top_marker("A1");
all_top_markers();