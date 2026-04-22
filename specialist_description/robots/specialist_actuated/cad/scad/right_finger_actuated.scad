// Collider of part right_finger_actuated on link right_finger, in millimeters, in the part frame.
// onshape_to_urdf.py reads only cube(), cylinder() and sphere(). The % mesh is a reference.

% scale(1000) import("right_finger_context.stl");

// Two boxes cover the top plate, split at the jaw notch. With one box, the inboard edge
// of the root would reach down the blade, and the jaw would close before the tips meet.

// The root box ends 7.00 mm from the pivot, on the side of the other finger. A longer box
// crosses the midline at some joint angle and touches the root box of the other finger. The
// gears mesh across the midline, and the base collider covers them.

translate([26.07, 2.58, -12.89])
  cube([41.14, 26.16, 9.78], center=true);

translate([33.69, 42.83, -12.89])
  cube([26.62, 54.34, 9.78], center=true);

// The underside below the pad is a ramp at 19.91 deg, so this box tilts. A flat box would
// hang 13.50 mm below the ramp at the fingertip. 15.00 mm is the most thickness that
// clears the plate boxes.

translate([28.50, 46.45, -33.20]) rotate([19.91, 0, 0])
  cube([7.00, 40.42, 15.00], center=true);
