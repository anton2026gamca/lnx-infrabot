$fn = 90;

text_font_size = 35;
height = 20;

module top_marker(mark = "0") {
    text_size = (len(mark) == 1) ? text_font_size : text_font_size * 2/3;
    difference() {
        translate([0,0,height])
            cylinder(d = 50, h = 2);
        translate([0,0,height + 1])
            linear_extrude(height = 1.01)
                text(str(mark),
                    size = text_size,
                    font = "Liberation Mono:style=Bold",
                    halign = "center",
                    valign = "center"
                );
    }
    rotate([0,0,53])difference() {
        cylinder(d = 27.2, h = height, $fn = 12);
        translate([0,0,-5])cylinder(d = 26.5, h = height + 10, $fn = 12);
    }
}

module top_char(mark = "0") {
    text_size = (len(mark) == 1) ? text_font_size : text_font_size * 2/3;
    translate([0,0,height + 1])
        linear_extrude(height = 1)offset(delta = -0.2)//font weight
            text(str(mark),
                size = text_size,
                font = "Liberation Mono:style=Bold",
                halign = "center",
                valign = "center"
            );
}

module all_top_markers() {
    parameters = ["1", "2", "3", "4", "A1", "A2", "B1", "B2"];
    for (A = [0:1:7]) {
        translate([55 * (A % 4 - 1.5),55* (sign(A-0.5 - 3))/2,0])
            top_marker(parameters[A]);
    }
}

module all_top_chars() {
    parameters = ["1", "2", "3", "4", "A1", "A2", "B1", "B2"];
    for (A = [0:1:7]) {
        translate([55 * (A % 4 - 1.5),55* (sign(A-0.5 - 3))/2,0])
            top_char(parameters[A]);
    }
}


all_top_markers();
translate([0,0,10])all_top_chars();