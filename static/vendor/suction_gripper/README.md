Elephant Robotics Pump 2.0 CAD
==============================

`pump_box.dae` and `pump_head.dae` are the official myCobot pump meshes from
Elephant Robotics' `mycobot_ros` repository:

https://github.com/elephantrobotics/mycobot_ros/tree/noetic/mycobot_description/urdf/mycobot_280_m5

The source repository is distributed under the BSD 3-Clause license. The
dashboard follows the official `mycobot_280m5_with_pump.urdf` placement:

- pump box: fixed to `g_base`, visual origin `(0, -0.15, 0)` meters and RPY
  `(pi/2, pi, 0)`;
- pump head: fixed to the J6 flange with its local +Y contact axis rotated by
  the official 1.579-radian mount angle, then clocked 90 degrees
  counterclockwise on the outward J6 face for the installed hose/head
  orientation (180 degrees from the previous rear-view interpretation).

Manufacturer nominal bounds are 72 x 52 x 37 mm for the pump box and
63 x 24.5 x 26.7 mm for the wrist head. The installed cup is rendered and
planned using the measured 22 mm diameter and 72 mm flange-to-contact length
(50 mm rigid head plus 22 mm free cup extension).

The source meshes declare millimeter units and Z-up axes. Their raw vertex
envelopes are 72 x 43 x 52 mm for the box and 24.5 x 63 x 26.7 mm for the
head. The pump-box mesh includes exterior features and does not reproduce the
published 37 mm nominal body dimension exactly; the application therefore
keeps the published dimensions as metadata rather than rescaling the CAD.

Elephant Robotics publishes a total accessory weight of 180 g and a rated
suction payload of 150 g. Neither the URDF nor published specifications contain
inertial data for the moving head. The physical center of mass is therefore
intentionally recorded as unknown; CAD geometry centers are not treated as COM.
