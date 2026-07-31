# Grasp Depth Repair and Cleanup Record

## Root cause

The former side-pinch planner placed the jaw-center TCP at 78% of the object's height plus a 4 mm compensation. For a 50.8 mm tall object, that selected approximately 43.6 mm above the table—only about 7.2 mm below the object top. The formula intentionally produced a shallow grip and did not express how much of the modeled finger actually overlapped the object.

The old coordinate configuration also retained several mutually exclusive paths (`z_lift`, `app_offset`, and `model_tcp`). The dashboard displayed a disabled “Legacy Flange Lift” value while new plans ignored it. Keeping those modes made it possible for calibration settings, diagnostics, and firmware coordinates to describe different tool points.

## Current height model

- Part `position.z` is the center of the part in robot-base meters.
- Object bottom/top are `centerZ ± height/2`.
- The desired jaw-center TCP is `objectCenterZ + pickHeightBiasM`.
- Pick Z Bias is limited to ±8 mm.
- The jaw target is raised only when necessary to keep the modeled 21 mm finger contact section above the configurable table clearance (4 mm default, bounded to 2–12 mm).
- The planned fingertip low point is `jawCenterZ - 21 mm`.
- Approach and lift targets are independent of grasp depth and remain at least 40 mm above the object top.
- The same TCP target is converted through the modeled flange/TCP transform for the firmware command. There is no separate physical-only descend adjustment.

Every pick now carries diagnostics for object bounds, requested and clamped jaw height, fingertip low point, desired/actual overlap, table clearance, TCP pose, flange pose, and outgoing firmware coordinates.

## Removed code

- Placeholder camera-frame detection ingestion and its `/api/camera/detections` and `/api/camera/accept-detections` routes. AprilTags are now the only automatic source of physical scene objects.
- Contour-created object normalization, semantic scan acceptance, camera-track suppression, robot-base-relative pixel projection, and their obsolete tests.
- Automatic classifier configuration on saved workcells. The small explicit single-visible-part classifier remains because it is user-triggered and cannot change geometry.
- Legacy planning/scene Z conversions that shifted objects by 30 mm.
- Runtime `z_lift`, `app_offset`, and `none` tool-offset modes, the Legacy Flange Lift UI, and redundant tool-offset fields in motion records. Old saved keys are consumed and discarded during migration.
- Untagged nearest-neighbor tracking, movement confirmation, and semantic proposal fields. The remaining filter is a three-frame median/wrapped-yaw filter keyed directly by AprilTag identity.

## Preserved intentionally

- AprilTag workspace localization, tagged-part registry, calibration wizard, camera-pose movement rejection, virtual-only parts, bins, programs, kinematics validation, robot feedback, safety gates, and saved-data migrations.
- Legacy `cameraToRobot` calibration data for backward-compatible loading and camera visualization. It does not create scene coordinates.
- The explicit `classify_visible_part` assistant action. It runs only on request and cannot alter coordinates, dimensions, tag binding, or scene presence.

## Offline verification

The regression suite exercises square boxes, rectangles, open boxes, multiple heights, workspace positions, yaw values including wraparound, table clearance, approach clearance, TCP/flange round trips, full-path host IK, elevated AprilTag planes, saved-data migration, and dashboard diagnostic wiring. It does not command physical robot motion.
