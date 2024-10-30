include <BOSL2/std.scad>
$fn=70;
pcb_width = 74;
pcb_height = 38;
esp_antenna_height = 7;
switch_indent = 26;
wall_thickness = 2;
box_heigth = 21;

power_switch_z_offset = 6 - (box_heigth/2 - wall_thickness);
power_switch_x_offset = 13.5;
power_switch_heigth = 4.0;
power_switch_width = 6.5;

usb_c_heigth = 5;
usb_c_width = 10;
usb_c_z_offset = 6 - (box_heigth/2 - wall_thickness);
usb_c_x_offset = 11.5;

joystick_y_offset = -21.5;
joystick_x_offset = -17;


module corner_cylinders(height, radius=2, real_pcb_width, real_pcb_heigth) {
    union() {
       translate([-(real_pcb_width/2 - wall_thickness), -(real_pcb_heigth/2 + 0.5 * wall_thickness), 0])
           cylinder(h=height, r=radius, center=true);
       
       translate([(real_pcb_width/2 - wall_thickness), -(real_pcb_heigth/2 + 0.5 * wall_thickness), 0])
           cylinder(h=height, r=radius, center=true);
       
       translate([-(real_pcb_width/2 - wall_thickness), (real_pcb_heigth/2 - esp_antenna_height), 0])
           cylinder(h=height, r=radius, center=true);

       translate([(real_pcb_width/2 - wall_thickness), (real_pcb_heigth/2 - esp_antenna_height), 0])
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
        translate([-pcb_width/2 + power_switch_width/2 - wall_thickness + power_switch_x_offset, pcb_height/2 - wall_thickness, power_switch_z_offset])
            #cube([power_switch_width, 2* wall_thickness, power_switch_heigth], center = true);
        translate([-pcb_width/2 + usb_c_width/2 - wall_thickness + usb_c_x_offset, -pcb_height/2 - 2 * wall_thickness, usb_c_z_offset])
            #cube([usb_c_width, 2*wall_thickness, usb_c_heigth], center = true);
    }
}
difference() {
    bottom_half() outer_armor();
    down((box_heigth - wall_thickness/2)/2) corner_cylinders(height = 2.5 + 2*wall_thickness, radius = 1.5, real_pcb_width=72, real_pcb_heigth=36);
    #down((box_heigth)/2) corner_cylinders(height = wall_thickness/2, radius = 3, real_pcb_width=72, real_pcb_heigth=36);
}
difference() {
    left (pcb_width + 3 * wall_thickness) difference() {
        union() {
        zrot(180) xrot(180) top_half() outer_armor();
            difference() {
                #down(wall_thickness) corner_cylinders(height = box_heigth - 2 * wall_thickness - 4, radius = 4, real_pcb_width=72, real_pcb_heigth=36);
                down(wall_thickness) corner_cylinders(height = box_heigth - 2 * wall_thickness - 4, radius = 1.5, real_pcb_width=72, real_pcb_heigth=36);
            }
        }
        translate([-pcb_width/2 - joystick_x_offset, pcb_height/2 + joystick_y_offset, -box_heigth + 24/2])
            #sphere(d=25);
    };
        left(pcb_width - pcb_width/4 - 3) back(12) down((box_heigth - wall_thickness)/2) #cylinder(h=wall_thickness, r=1.5, center=true);
        left (pcb_width + 3 * wall_thickness) yflip() yrot(180) xrot(180) top_half() outer_armor();
}
