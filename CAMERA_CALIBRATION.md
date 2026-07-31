# Fiducial camera setup

1. Print every file in `data/fiducial_assets` at 100% scale. Confirm each workspace tag is exactly 50 mm square with a ruler.
2. Mount the Lenovo camera rigidly above the table. Lock its position before calibration.
3. Start the dashboard camera and select **Calibrate Camera**.
4. Hold the ChArUco board at varied positions, scales, and tilts. Capture at least 12 accepted samples spanning the center, four image regions, two noticeably different distances, and four tilted views. Use **Remove Last Photo** when a weak view is accepted. Practical mode accepts lens RMS ≤2.5 px and worst-view error ≤4 px.
5. Attach tags 0–3 near the workspace corners, flat on the table and outside the object area.
6. Measure each tag center from the robot-base origin in meters using +X forward and +Y left. Enter X, Y, and tag yaw, then select **Save Marker Map**.
7. Select **Check Workspace Tags**. All four markers must be visible and each must contribute at least two corner inliers; practical mode allows RMS reprojection ≤10 px, maximum ≤18 px, and requires coverage ≥12%.
8. Select **Accept Camera Pose** to save the baseline homography.
9. Place one visible test target at nine distributed known robot XY positions and use **Read Camera** for every row.
10. Keep the target stationary while the wizard records five stability frames. Continuous tracking is available only after ≤10 mm RMS, ≤20 mm maximum XY error, and ≤5 mm stationary spread all pass.

If the camera moves more than the allowed baseline drift, localization rejects frames until **Accept Camera Pose** is used again. Invalid frames never replace existing scene detections.

Tagged objects can be configured through `calibration.fiducials.objectTags`; each entry supports `id`, `label`, `class`, `size`, `centerOffsetM`, and `yawOffsetDeg`.
