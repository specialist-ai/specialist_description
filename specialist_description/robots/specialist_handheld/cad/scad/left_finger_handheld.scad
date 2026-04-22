// Collider of part left_finger_handheld on link left_finger, in millimeters, in the part frame.
// onshape_to_urdf.py reads only cube(), cylinder() and sphere(). The % mesh is a reference.

% scale(1000) import("left_finger_context.stl");

// Two boxes cover the top plate, split at the jaw notch. With one box, the inboard edge
// of the root would reach down the blade, and the jaw would close before the tips meet.

// The root box ends 7.00 mm from the pivot, on the side of the other finger. A longer box
// crosses the midline at some joint angle and touches the root box of the other finger. The
// gears mesh across the midline, and the base collider covers them.

translate([-26.075, 2.47, -12.89])
  cube([41.15, 26.20, 9.78], center=true);

translate([-33.52, 42.79, -12.89])
  cube([26.96, 54.42, 9.78], center=true);

// The underside below the pad is a ramp at 19.91 deg, so this box tilts. A flat box would
// hang 13.50 mm below the ramp at the fingertip. 15.00 mm is the most thickness that
// clears the plate boxes.

translate([-28.50, 46.45, -33.20]) rotate([19.91, 0, 0])
  cube([7.00, 40.42, 15.00], center=true);
