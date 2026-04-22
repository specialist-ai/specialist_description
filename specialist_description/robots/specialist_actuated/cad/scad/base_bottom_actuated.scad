// Collider of part base_bottom_actuated on link base, in millimeters, in the part frame.
// onshape_to_urdf.py reads only cube(), cylinder() and sphere(). The % mesh is a reference.

% scale(1000) import("base_context.stl");

// Base, screws and servo

translate([-0.00, -24.84, -25.61]) rotate([95.92, -0.15, 179.99])
  cube([50.15, 59.02, 79.49], center=true);

// Mount

translate([-0.00, -15.68, -68.33]) rotate([-180.00, 70.00, 90.00])
  cube([55.00, 56.00, 74.71], center=true);

// Camera and protector

translate([0.00, -33.87, 9.58]) rotate([180.00, 20.00, 90.00])
  cube([41.50, 56.00, 119.00], center=true);
