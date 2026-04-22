// Collider of part base_bottom_handheld on link base, in millimeters, in the part frame.
// onshape_to_urdf.py reads only cube(), cylinder() and sphere(). The % mesh is a reference.

% scale(1000) import("base_context.stl");

// Base top and screws

translate([-0.00, -5.00, -14.00]) rotate([0.00, 90.00, 0.00])
  cube([28.00, 38.50, 50.00], center=true);

// Base bottom

translate([-0.00, 0.95, -24.16]) rotate([179.91, 69.99, 89.91])
  cube([16.65, 50.00, 28.53], center=true);

translate([0.00, -30.91, -20.05]) rotate([179.91, 69.99, 89.91])
  cube([46.16, 50.02, 28.53], center=true);

translate([0.00, -52.52, -44.06]) rotate([179.91, 69.99, 89.91])
  cube([15.83, 50.01, 28.53], center=true);

// Camera

translate([-0.00, -37.40, 7.94]) rotate([-180.00, 20.00, 90.00])
  cube([22.75, 46.00, 113.50], center=true);

// Camera lens. The dome is a sphere, and a box of the same span has twice its volume.

translate([-0.00, -24.99, 42.02])
  sphere(15.81);
