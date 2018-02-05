
DEBUG = 0;

tolerance = 0.20;
tolerance2 = 2*tolerance;
tolerance4 = 4*tolerance;

sensor_radius = 15.5;
beeper_radius = 7.3;
beeper_trans = [20, 13, -14];

green_pcb_l  = 61.0;    // green PCB length
green_pcb_w  = 54.0;    // green PCB width
green_pcb_h  = 1.6;     // green PCB thickness
green_offset = 3.0;

red_pcb_l    = 66.5;           // red PCB length
red_pcb_w    = green_pcb_w;    // red PCB width
red_pcb_h    = 1.55;           // red PCB thickness
red_offset   = 6.0;

pcb_dist0    = 12.7;     // inner distance between PCBs
pcb_dist     = pcb_dist0 + ((green_pcb_h + red_pcb_h) / 2);

box_thickness = 1.6;

box_y_red   = red_offset;               // red PCB position
box_y_green = box_y_red + pcb_dist;     // green PCB position
box_h_i = box_y_green + green_offset;
box_h_o = box_h_i + 2*box_thickness;

box_w_i = green_pcb_w + tolerance2;   // inner box width
box_w_o = box_w_i + 2*box_thickness;  // outer box width
box_l = red_pcb_l + 2*box_thickness;  // outer box length

pcb_guard_h = 1;
pcb_guard_w = box_thickness + 2;

pcb_guard_x1 = box_w_i + box_thickness - pcb_guard_w;
pcb_guard_x2 = pcb_guard_x1 + 1;

pcb_guard_y1 = box_y_red - pcb_guard_h;
pcb_guard_y2 = box_y_red + red_pcb_h + tolerance2;
pcb_guard_y3 = box_y_green - pcb_guard_h;
pcb_guard_y4 = box_y_green + green_pcb_h + tolerance2;

cover_flap_y1 = pcb_guard_y2 + tolerance4;
cover_flap_y2 = pcb_guard_y3 - tolerance4 - pcb_guard_h;
cover_flap_h  = cover_flap_y2 - cover_flap_y1;
flap_l = 15;

// the 2D profile of the box...

// green PCB incl. tolerance
//
module
green_PCB_2D()
{
   color([0, 1, 0])
   square([green_pcb_w + tolerance2, green_pcb_h + tolerance2]);
}

// red PCB incl. tolerance
//
module
red_PCB_2D()
{
   color([1, 0, 0])
      square([red_pcb_w + tolerance2, red_pcb_h + tolerance2]);
   color([1, 0.5, 0.5])
   translate([10, red_pcb_h - 5, 0])
      square([red_pcb_w - 20 + tolerance2, 5]);
}

module pcb_guard()
{
   square([pcb_guard_w, pcb_guard_h]);
}

module rounded_rect(w, h, r)
{
   translate([r,r])
   minkowski($fn=50)
      {
         square([w - 2*r, h - 2*r]);
         circle(r);
      }
}

module
profile_2D()
{
   difference()
      {
        square([box_w_o, box_h_o]);
        translate([box_thickness, box_thickness, 0])
           rounded_rect(box_w_i, box_h_i, 1);
      }

   if (DEBUG)
      {
        translate([box_thickness, box_y_green, 0]) green_PCB_2D();
        translate([box_thickness, box_y_red, 0])   red_PCB_2D();
      }

   translate([0, pcb_guard_y1, 0])   pcb_guard();
   translate([0, pcb_guard_y2, 0])   pcb_guard();
   translate([0, pcb_guard_y3, 0])   pcb_guard();
   translate([0, pcb_guard_y4, 0])   pcb_guard();

   translate([pcb_guard_x1, pcb_guard_y1, 0])   pcb_guard();
   translate([pcb_guard_x1, pcb_guard_y2, 0])   pcb_guard();
   translate([pcb_guard_x2, pcb_guard_y3, 0])   pcb_guard();
   translate([pcb_guard_x2, pcb_guard_y4, 0])   pcb_guard();
}

module profile_3D()
{
   linear_extrude(height = red_pcb_l + 10,
                  center = false,
                  convexity = 10,
                  twist = 0) profile_2D();
}

/// the bottom, left, right, front, and back of the box.
/// the top (cover) of the box is printed separately.
module box_3D()
{
   // add bottom
   //
   translate(0, 0, box_thickness)   profile_3D();
   cube([box_w_o, box_h_o, box_thickness]);
   translate(beeper_trans)   cylinder(h = 20, r = beeper_radius + 1, $fn = 90);

   translate([box_w_o/2, 0, 20])
   rotate([-90, 0, 0])
   cylinder(h = 5, r = sensor_radius + 1, $fn = 90);
}

/// the box with cutouts for the sensor and the beeper
module box_3D_cutout()
{
   difference()
      {
        box_3D();
        union()
           {
             // after rotate(-90, 0, 0)) below, in the standard view:
             //
             // x: left-right
             // y: front-back
             // z: up-down

             // sensor cutout
             //
             translate([box_w_o/2, 0, 20])
             rotate([-90, 0, 0])
             cylinder(h = 6, r = sensor_radius, $fn = 90);

             // beeper cutout
             //
             translate(beeper_trans)
             cylinder(h = 20, r = beeper_radius, $fn = 90);
           }
      }
}

/// a cube with rounded edges (to be intersected with the box)
module rounded_block(r)
{
   translate([r,r,r])
   minkowski($fn=50)
      {
        cube([box_w_o - 2*r, box_h_o - 2*r, box_l + 10]);
        sphere(r);
      }
}

/// the box except its cover
module main_body()
{
   intersection()
      {
        box_3D_cutout();
        rounded_block(1);
      }
}

// the cover for the box before rounding its edges
module cover_3D()
{
   cube([box_w_o, box_h_o, box_thickness]);

   translate([5, box_thickness, box_thickness])
      cube([box_w_o - 10, 2, 2]);
   translate([5, box_h_o - box_thickness - 2, box_thickness])
      cube([box_w_o - 10, 2, 2]);

   translate([box_thickness, cover_flap_y1, box_thickness])
      cube([2, cover_flap_h, flap_l]);

   translate([box_w_o - box_thickness - 2, cover_flap_y1, box_thickness])
      cube([2, cover_flap_h, flap_l]);

}

module cover_3D_cutout()
{
switch_x = 5;
switch_w = 11;
switch_y = box_y_green + 1.0;
switch_h = 4;

led_dia = 3.5;
led_radius = 0.5 * led_dia;

red_led_x = 20.7;    // center
red_led_y = box_y_green - led_radius;

green_led_x = red_led_x + 5.12;
green_led_y = red_led_y;   // center

prog_x = green_led_x + 3.3;
prog_w = 8.62 + tolerance2;
prog_y = box_y_green - 3.1;
prog_h = 6.1;

   difference()
      {
        cover_3D();

        // the translates are mirrored at the x axis, therefore
        // box_h_o - yyy ...

        // cutout for the on-off switch
        //
        translate([switch_x, box_h_o - switch_y, -5])
           cube([switch_w, switch_h, 10]);

        // cutout for the red LED
        //
        translate([red_led_x, box_h_o - red_led_y, -5])
             cylinder(h = 10, r = led_radius, $fn = 90);

        // cutout for the green LED
        //
        translate([green_led_x, box_h_o - green_led_y, -5])
             cylinder(h = 10, r = led_radius, $fn = 90);

        // cutout for the program connector
        //
        translate([prog_x, box_h_o - prog_y, -5])
           cube([prog_w, prog_h, 10]);
      }
}

module main_cover()
{
   intersection()
      {
        cover_3D_cutout();
        rounded_block(1);
      }
}

/// the body or the cover of the box. Uncomment onbe of the two lines below
   main_body();
// main_cover();

