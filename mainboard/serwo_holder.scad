// Function to create a plate with rounded corners
module base(length, width, height, corner_radius) {
    minkowski() {
        cube([length, width, height/2], center=false);
        cylinder(d=corner_radius, h=height/2, $fn=32);
    };
}

module cylinder_set(d_insert_hole, centers, height) {
    for (center = centers) {
        translate([center[0], center[1], 0])
            cylinder(h=height, d=d_insert_hole, $fn=32);
    }
}

module drill(drill_depth, diameter) {
    tab_width = 5;
    width = 23.3;
    translate([0, 0, 0])
        rotate([0, 90, 90])
            cylinder(d=diameter, h=drill_depth, $fn=20);
    translate([width + tab_width +0.3, 0, 0])
        rotate([0, 90, 90])
            cylinder(d=diameter, h=drill_depth, $fn=20);
}

// Servo module
module servo(clarence) {
    tab_length = 2.4;
    tab_width = 5;
    length = 24.3;
    from_fron_shift = length - (17.5 + tab_length);
    width = 23.3;
    height = 12.4;
    union() {
        union() {
            cube([width+clarence, length+clarence, height+clarence]);
            translate([-tab_width, from_fron_shift, 0])
                cube([tab_width+clarence, tab_length+clarence, height+clarence]);
            translate([width, from_fron_shift, 0])
                cube([tab_width+clarence, tab_length+clarence, height+clarence]);
        }
        translate([-2.5, length - 5, 0])
        cube([2.5, 5, height]);
    }
}
// Example usage
over_length = 6;
clarence = 0.3;
length = 46;
width = 34;
height = 10;
d_insert_hole = 4.6;
corner_radius = 4;

centers = [
    [0, 29.7],
    [19.3, 0],
    [32.9, 34],
    [46, 5.5]
];

difference() {
    translate([-over_length, 0, 0])
        base(length + over_length, width, height, corner_radius);
    cylinder_set(d_insert_hole, centers, 3 * height/4);
    translate([0, 0, height/2])
        cylinder_set(d_insert_hole / 2, centers, height/2);
translate([-corner_radius/2,2,-(12.4-height)/2])
    servo(clarence);
translate([-corner_radius-0.5,-5 ,height/2])
    drill(12, 4.5);
translate([-corner_radius-0.5, 5 ,height/2])
    #drill(14.5, 3);
translate([-2,-5 ,0])
    #cube([3 * 23.3 /4, 10,  height]);
}


