include <BOSL2/std.scad>
$fn=70;
pcb_width = 72;
pcb_height = 36;
esp_antenna_height = 6;
switch_indent = 25;
wall_thickness = 2;
box_heigth = 18;

power_switch_z_offset = 4 - (box_heigth/2 - wall_thickness);
power_switch_x_offset = 10;
power_switch_heigth = 3;
power_switch_width = 6;

usb_c_heigth = 4;
usb_c_width = 10;
usb_c_z_offset = 4 - (box_heigth/2 - wall_thickness);
usb_c_x_offset = 9;

joystick_y_offset = -16;
joystick_x_offset = -15;


module corner_cylinders(height, radius=2) {
    union() {
       translate([-(pcb_width/2 - wall_thickness), -(pcb_height/2 + 0.5 * wall_thickness), 0])
           cylinder(h=height, r=radius, center=true);
       
       translate([(pcb_width/2 - wall_thickness), -(pcb_height/2 + 0.5 * wall_thickness), 0])
           cylinder(h=height, r=radius, center=true);
       
       translate([-(pcb_width/2 - wall_thickness), (pcb_height/2 - esp_antenna_height), 0])
           cylinder(h=height, r=radius, center=true);

       translate([(pcb_width/2 - wall_thickness), (pcb_height/2 - esp_antenna_height), 0])
           cylinder(h=height, r=radius, center=true);
    }
}


module outer_armor() {
    difference() {
        cube([pcb_width + 2 * wall_thickness, pcb_height + 2 * wall_thickness + esp_antenna_height, box_heigth], center=true);
        fwd(esp_antenna_height/2)
            cube([pcb_width, pcb_height, box_heigth - (2 * wall_thickness)], center = true);
        translate([pcb_width/2 - switch_indent + wall_thickness, pcb_height/2, 0])
            cube([pcb_width - switch_indent, esp_antenna_height, box_heigth - (2 * wall_thickness)], center = true);
        translate([-pcb_width/2 + switch_indent/2 - wall_thickness, pcb_height/2 + wall_thickness, 0])
            cube([switch_indent, esp_antenna_height, box_heigth], center = true);
        translate([-pcb_width/2 + power_switch_width/2 - wall_thickness + power_switch_x_offset, pcb_height/2 - esp_antenna_height + 2 * wall_thickness, power_switch_z_offset])
            #cube([power_switch_width, wall_thickness, power_switch_heigth], center = true);
        translate([-pcb_width/2 + usb_c_width/2 - wall_thickness + usb_c_x_offset, -pcb_height/2 - 2 * wall_thickness, usb_c_z_offset])
            #cube([usb_c_width, wall_thickness, usb_c_heigth], center = true);
    }
}
difference() {
    union() {
        bottom_half() outer_armor();
        down(2*wall_thickness + 2.5) corner_cylinders(height = 2.5, radius = 4);
    }
    down(2*wall_thickness + 3) corner_cylinders(height = 2.5 + 2*wall_thickness, radius = 1.5);
    #down(3*wall_thickness + 2.5) corner_cylinders(height = 1, radius = 3.2);

}
left (pcb_width + 3 * wall_thickness) difference() {
    union() {
    zrot(180) xrot(180) top_half() outer_armor();
        difference() {
            #down(wall_thickness) corner_cylinders(height = box_heigth - 2 * wall_thickness - 4, radius = 4);
            down(wall_thickness) corner_cylinders(height = box_heigth - 2 * wall_thickness - 4, radius = 1.5);
        }
    }
    translate([-pcb_width/2 - joystick_x_offset, pcb_height/2 + joystick_y_offset, -box_heigth - wall_thickness + 24/2])
        #sphere(d=24);
}

