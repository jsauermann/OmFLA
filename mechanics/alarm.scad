
// variables that may be changed by the user
DEBUG  = 0;   // debug mode
UPPER  = 1;   // print the upper or lower half of the box
BUCKLE = 0;   // print the body with left and right belt loops

// variables that ahould not be changed by the user

tb_cut_z = 50;

/// colors
c_green = [0, 1, 0];
c_red   = [1, 0, 0];

tolerance = 0.20;
tolerance2 = 2*tolerance;
tolerance4 = 4*tolerance;

sensor_radius = 15.5;

beeper_radius = 7.3;
beeper_trans = [20, 13, -14];

antenna_radius = 1.5;
antenna_trans = [42, 13, -14];

green_pcb_l  = 61.0;    // green PCB length
green_pcb_w  = 54.0;    // green PCB width
green_pcb_h  = 1.6;     // green PCB thickness
green_offset = 4.0;

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
box_l_i = red_pcb_l + 5;              // inner box length
box_l_o = box_l_i + 2*box_thickness;  // outer box length

pcb_guard_h = 1;
pcb_guard_w = box_thickness + 2;

pcb_guard_x1 = box_w_i + box_thickness - pcb_guard_w;
pcb_guard_x2 = pcb_guard_x1 + 1;

pcb_guard_y1 = box_y_red - pcb_guard_h;
pcb_guard_y2 = box_y_red + red_pcb_h + tolerance2;
pcb_guard_y3 = box_y_green - pcb_guard_h;
pcb_guard_y4 = box_y_green + green_pcb_h + tolerance2;

// the wide flap that connects the upper and lower halves of the box
//
flap_w = 8;
flap_h = 1.4;
flap1_l = 30;
flap2_l =  8;

module flap_2D(mirror)
{
flap_w2 = 0.5*flap_w;
flap_w2h = flap_w2 - flap_h;

   if (mirror)
      polygon([[-flap_w2,  0],
               [-flap_w2,  flap_h],
               [ flap_w2h, flap_h],
               [ flap_w2,  0]]);
   else
      polygon([[-flap_w2,  0],
               [-flap_w2,  flap_h],
               [ flap_w2,  flap_h],
               [ flap_w2h, 0]]);
}
module flap1(mirror)
{
   rotate([0, 90, 0])
   linear_extrude(height = flap1_l,
                  center = false,
                  convexity = 10,
                  twist = 0)   flap_2D(mirror);
}

// the narrow flap that connects the upper and lower halves of the box
//
module flap2(mirror)
{
   rotate([0, 90, 90])
   linear_extrude(height = flap2_l,
                  center = false,
                  convexity = 10,
                  twist = 0)   flap_2D(mirror);
}

// green PCB incl. tolerance
//
module
green_PCB_2D()
{
   color(c_green)
   square([green_pcb_w + tolerance2, green_pcb_h + tolerance2]);
}

// red PCB incl. tolerance
//
module
red_PCB_2D()
{
   color(c_red)
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
   linear_extrude(height = box_l_i,
                  center = false,
                  convexity = 10,
                  twist = 0) profile_2D();
}

/// the bottom, left, right, front, and back of the box.
/// the top (cover) of the box is printed separately.
module box_3D()
{
   // bottom plate
   //
   cube([box_w_o, box_h_o, box_thickness]);

   // ring around the beeper in the bottom plate
   //
   translate(beeper_trans)    cylinder(h = 17, r = beeper_radius + 1, $fn = 90);

   // main body
   //
   translate([0, 0, box_thickness])   profile_3D();

   // ring around the sensor in the main body
   //
   translate([box_w_o/2, 0, 20])
   rotate([-90, 0, 0])
   cylinder(h = 5, r = sensor_radius + 1, $fn = 90);

   // top plate
   //
   translate([0, 0, box_thickness + box_l_i])
      cube([box_w_o, box_h_o, box_thickness]);
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

             // antenna cutout
             //
             translate(antenna_trans)
             cylinder(h = 20, r = antenna_radius, $fn = 90);

             translate([0, 0.5*box_h_o, box_l_o - 5])
             mirror([0, 1, 0])
             translate([0, -0.5*box_h_o, 0])
             cover_3D_cutout();
           }
      }
}

/// a cube with rounded edges (to be intersected with the box)
module rounded_block(r)
{
   translate([r,r,r])
   minkowski($fn=50)
      {
        cube([box_w_o - 2*r, box_h_o - 2*r, box_l_o - 2*r]);
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
   // base plate
   //
   cube([box_w_o, box_h_o, box_thickness]);

   // guard next to green PCB
   //
   translate([5, box_thickness, box_thickness])
      cube([box_w_o - 10, 2, 5]);

   // guard next to red PCB
   //
   translate([5, box_h_o - box_thickness - 2, box_thickness])
      cube([box_w_o - 10, 2, 5]);
}

module cover_2D_cutout()
{
switch_w = 11;
switch_h = 7;
switch_x = 5.5;
switch_y = box_y_green + 1.5;

led_dia = 3.5;
led_radius = 0.5 * led_dia;

red_led_x = 20.7;    // center
red_led_y = box_y_green - led_radius;

green_led_x = red_led_x + 5.12;
green_led_y = red_led_y;   // center

prog_w = 10.24 + tolerance2;
prog_h = 4.1;
prog_x = green_led_x + 4;
prog_y = box_y_green - 4.1;

   // cutout for the on-off switch
   //
   translate([switch_x, box_h_o - switch_y, -5])
      square([switch_w, switch_h]);

   // cutout for the red LED
   //
   translate([red_led_x, box_h_o - red_led_y, -5])
      circle(r = led_radius, $fn = 90);

   // cutout for the green LED
   //
   translate([green_led_x, box_h_o - green_led_y, -5])
      circle(r = led_radius, $fn = 90);

   // cutout for the program connector
   //
   translate([prog_x, box_h_o - prog_y, -5])
      square([prog_w, prog_h]);
}

module cover_3D_cutout()
{
   linear_extrude(height = 20,
                  center = false,
                  convexity = 2,
                  twist = 0) cover_2D_cutout();
}

///  a (left or right) buckle for holding the casing
module buckle_2D(rnd)
{
l  = 24;
h1 =  6;
dh = 1;
h2 = h1 + dh;
wi =  1.141*(rnd + dh);
   polygon([[ 0,  0],               [wi,  0],
            [wi + h1, h1],          [wi + h1 + l, h1],
            [wi + h1 + l + h1, 0],  [wi + h1 + l + h1 + wi,  0],
            [wi + h1 + l + h1 + wi - h2, h2],          [h2, h2]]);
}

module buckle_3D()
{
rnd = 1;
   translate([0, 0, rnd])
   minkowski($fn=20)
      {
        linear_extrude(height = 15,
                       center = false,
                       convexity = 2,
                       twist = 0) buckle_2D(rnd);

        sphere(rnd);
      }
}

module main_body_and_buckle()
{
   main_body();

   if (BUCKLE)
      {
        // left buckle
        //
        translate([0.5, 22, 3])
        rotate([0, -90, 90])
        buckle_3D();

        // right buckle
        //
        translate([box_w_o - 0.5, 5, 3])
        rotate([0, -90, -90])
        buckle_3D();
      }
}

/// either the top or the bottom of the box.

intersection()
{
   main_body_and_buckle();
   if (UPPER)
      translate([0, 0, tb_cut_z])
      cube([100, 100, 100]);
   else
      translate([-40, 0, 0])
      cube([150, 100, tb_cut_z]);
}

if (UPPER)
{
   // wide flap (front)
   //
   translate([0.5*(box_w_o - flap1_l),
              box_thickness,
              tb_cut_z])   flap1(0);

   // wide flap (back)
   //
   translate([0.5*(box_w_o - flap1_l),
              box_h_o - (flap_h + box_thickness),
              tb_cut_z])   flap1(1);

   // narrow flap (left)
   //
   translate([box_thickness,
              0.5*(box_h_o - flap_w),
              tb_cut_z])   flap2(1);

   // narrow flap (right)
   //
   translate([box_w_o - box_thickness,
              0.5*(box_h_o - flap_w),
              tb_cut_z])   flap2(0);
}
