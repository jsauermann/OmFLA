
// variables that may be changed by the user
DEBUG  = 0;   // debug mode
UPPER  = 1;   // print the upper or lower half of the box
BUCKLE = 1;   // print the body with left and right belt loops
LEDS   = 1;   // do an LED cutout

// variables that ahould not be changed by the user

tb_cut_z = 50;

/// colors
c_green = [0, 1, 0];
c_red   = [1, 0, 0];

tolerance = 0.20;
tolerance2 = 2*tolerance;
tolerance4 = 4*tolerance;

sensor_radius = 15.5;

beeper_radius = 7.4;
beeper_trans = [21.5, 16.9, -14];

antenna_radius = 1.5;
antenna_trans = [42, 16.9, -14];

green_pcb_l  = 61.0;    // green PCB length
green_pcb_w  = 54.0;    // green PCB width
green_pcb_h  = 1.6;     // green PCB thickness
green_offset = 4.0;

red_pcb_l    = 66.5;           // red PCB length
red_pcb_w    = green_pcb_w;    // red PCB width
red_pcb_h    = 1.55;           // red PCB thickness
red_offset   = 6.0;

pcb_dist0    = 11.2;     // inner distance between green PCB and red PCB
pcb_dist     = pcb_dist0 + red_pcb_h;

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
flap3_l =  6;

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

module flap3(mirror)
{
   rotate([0, 90, 0])
   linear_extrude(height = flap3_l,
                  center = false,
                  convexity = 10,
                  twist = 0)   flap_2D(mirror);
}

// green PCB incl. tolerance
//
module
green_PCB()
{
   cube([
          green_pcb_w + tolerance2,
          green_pcb_h + tolerance2,
          green_pcb_l
        ]);
}

// red PCB incl. tolerance
//
module
red_PCB()
{
      cube([
             red_pcb_w + tolerance2,
             red_pcb_h + tolerance2,
             red_pcb_l
           ]);

      translate([10, red_pcb_h - 5, 0])
         cube([
                red_pcb_w - 20 + tolerance2,
                5,
                red_pcb_l
              ]);
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

   // 1mm thick ring around the beeper in the bottom plate
   //
   translate(beeper_trans)    cylinder(h = 17, r = beeper_radius + 1, $fn = 90);

   // main body
   //
   translate([0, 0, box_thickness])   profile_3D();


   if (DEBUG)
      {
        color(c_green)
        translate([box_thickness, box_y_green, box_thickness])
        green_PCB();

        color(c_red)
        translate([box_thickness, box_y_red, box_thickness])
        red_PCB();
      }

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
    //       mirror([0, 1, 0])
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
tol = 0.5;   // a little more than needed
tol2 = 2*tol;

leds_w   = 4.3 + tol2;
leds_h   = 9.7 + tol2;

prog_w   = 8.1 + tol2;
prog_h   = 5.0 + tol2;

switch_w = 7.6 + tol2;
switch_h = 3.5 + tol2;

leds_wh   = [leds_w,   leds_h];
prog_wh   = [prog_w,   prog_h];
switch_wh = [switch_w, switch_h];

// distances from left green PCB edge
//
gpcb_leds   =  4.9;
gpcb_prog   = 19.1;
gpcb_switch = 35.3;
gpcb_outer  =  2.0;   // left green PCB edge to box hull

leds_x   = gpcb_leds   + gpcb_outer - tol;
prog_x   = gpcb_prog   + gpcb_outer - tol;
switch_x = gpcb_switch + gpcb_outer - tol;

leds_y   = box_y_green - 0.8 + tol;
prog_y   = box_y_green - 0.8 + tol;
switch_y = box_y_green - 2.0 + tol;

   // cutout for the LED block
   //
   if (LEDS)
      {
        translate([leds_x, box_h_o - leds_y])
        square(leds_wh, center = false);
      }

   // cutout for the program connector
   //
   translate([prog_x, box_h_o - prog_y])
      square(prog_wh, center = false);

   // cutout for the on-off switch
   //
   translate([switch_x, box_h_o - switch_y])
      square(switch_wh, center = false);
}

module cover_3D_cutout()
{
   linear_extrude(height = 20,
                  center = false,
                  convexity = 2,
                  twist = 0) cover_2D_cutout();
}

/// a buckle for holding the housing, e.g. by means of a tourniquet
module buckle_2D(rnd)
{
l  = 35;
h1 =  3;
dh = 0.5;
h2 = h1 + dh;
wi =  1.141*(rnd + dh);
   polygon([[ 0,  0],               [wi,  0],
            [wi + h1, h1],          [wi + h1 + l, h1],
            [wi + h1 + l + h1, 0],  [wi + h1 + l + h1 + wi,  0],
            [wi + h1 + l + h1 + wi - h2, h2],          [h2, h2]]);
}

module buckle_3D()
{
rnd = 1;      // minkowski rounding diameter
width = 40;   // width of the buckle
   translate([0, -1.5, rnd])
   minkowski($fn=20)
      {
        linear_extrude(height = width,
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
        // buckle
        //
        translate([50, 27, 3])
        rotate([0, -90, 0])
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


   // broken wide flaps (back)
   //
   translate([box_thickness + 1.0,
              box_h_o - (flap_h + box_thickness),
              tb_cut_z])   flap3(1);

   translate([box_w_o - (box_thickness + 1.0 + flap3_l),
              box_h_o - (flap_h + box_thickness),
              tb_cut_z])   flap3(1);

   // narrow flap (left)
   //
   translate([box_thickness + flap_h,
              0.5*(box_h_o - flap_w),
              tb_cut_z])   flap2(1);

   // narrow flap (right)
   //
   translate([box_w_o - box_thickness,
              0.5*(box_h_o - flap_w),
              tb_cut_z])   flap2(0);
}
