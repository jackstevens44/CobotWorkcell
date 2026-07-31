# CobotWorkcell

> An open-source, camera-aware control and programming platform for the Elephant Robotics myCobot 280 M5.

[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-D22128.svg)](LICENSE)
[![Offline CI configured](static/docs/offline-ci-badge.svg)](https://github.com/jackstevens44/CobotWorkcell/actions/workflows/ci.yml)
[![Robot](https://img.shields.io/badge/Robot-myCobot%20280%20M5-2563EB)](https://www.elephantrobotics.com/en/mycobot-en/)
[![Control](https://img.shields.io/badge/Control-Local%20first-0F766E)](#privacy)
[![Tests](https://img.shields.io/badge/Offline%20tests-149%20passing-15803D)](#testing)

> ⚠️ **Early development:** This project is under active development and is not a safety-rated robot controller. Expect changes, test at reduced speed, and supervise every physical motion.

**Initial launch:** July 30, 2026

CobotWorkcell combines a browser-based digital twin, a visual robot programmer, deterministic AprilTag object tracking, calibrated tool geometry, guarded robot jogging, spatial AI commands, and independently validated pick-and-place planning for the Elephant Robotics myCobot 280 M5.

It is designed for one small tabletop workcell and one important safety principle:

> **Planning, camera localization, editing, and simulation may run offline. Physical motion requires a fresh validated preview and explicit operator confirmation.**

This document is the complete setup, operation, calibration, programming, troubleshooting, development, API, and maintenance guide. Read the [Safety](#safety) section before connecting a physical robot.

[Contributing](CONTRIBUTING.md) · [Security](SECURITY.md) · [Support](SUPPORT.md) · [Code of Conduct](CODE_OF_CONDUCT.md)

---

## Contents

Every entry below is clickable. On GitHub, the README’s document-outline control also keeps the same headings available as side navigation.

**Start here**

- [Quick start](#quick-start)
- [Capabilities](#capabilities)
- [Safety](#safety)
- [Hardware and compatibility](#hardware-and-compatibility)
- [Understanding coordinates](#understanding-coordinates)

**Using the platform**

- [Connect the physical robot](#connect-the-physical-robot)
- [Dashboard tour](#dashboard-tour)
- [Camera and AprilTag setup](#camera-and-apriltag-setup)
- [Parts, bins, and points](#parts-bins-and-points)
- [End effectors](#end-effectors)
- [Tool contact calibration](#tool-contact-calibration)
- [Creating programs](#creating-programs)
- [Simulation and physical execution](#simulation-and-physical-execution)
- [Spatial AI assistant](#spatial-ai-assistant)

**Reference**

- [Data, backups, and privacy](#data-backups-and-privacy)
- [Configuration](#configuration)
- [Command line](#command-line)
- [HTTP API](#http-api)
- [Architecture](#architecture)
- [Repository structure](#repository-structure)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Repository automation](#repository-automation)
- [Public release checklist](#public-release-checklist)
- [Contributing](#contributing)
- [Known limitations](#known-limitations)
- [ROS 2](#ros-2)
- [License and third-party assets](#license-and-third-party-assets)
- [Acknowledgements](#acknowledgements)
- [Final operator checklist](#final-operator-checklist)

---

## Quick start

The project has been developed and tested on macOS.

### 1. Download and install

```bash
git clone https://github.com/jackstevens44/CobotWorkcell.git
cd CobotWorkcell

python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

### 2. Add an OpenAI key

The key is optional unless you want the AI assistant.

```bash
cp api_keys.env.example api_keys.env
```

Open `api_keys.env` and replace the example value:

```text
OPENAI_API_KEY=sk-your-key-here
OPENAI_REALTIME_MODEL=gpt-realtime-2.1
OPENAI_REALTIME_VOICE=marin
```

`api_keys.env` is ignored by Git. Never commit the real key.

### 3. Start the platform

```bash
python3 web_server.py --host 127.0.0.1 --web-port 8768
```

Open [http://127.0.0.1:8768](http://127.0.0.1:8768).

The dashboard works without a robot or camera. To use hardware, select the robot connection in the dashboard, start the external camera, and follow the calibration guide before physical motion.

---

## Capabilities

| Capability | What it provides |
| --- | --- |
| Live 3D workcell | Robot feedback, tools, parts, bins, points, camera, and validated paths |
| Robot programming | Joint Move, Linear Move, Home, Pick, Place, Tool, and Wait commands |
| Jogging and point capture | Joint/TCP jogging, hand guiding, embedded waypoints, and reusable points |
| AprilTag tracking | Stable identity, XY position, yaw, dimensions, visibility, and tag offsets |
| Camera calibration | Guided ChArUco calibration and permanent workspace-tag localization |
| End effectors | Adaptive gripper and base-mounted Pump 2.0 suction accessory |
| Motion validation | Firmware IK/FK plus independent host IK/FK and complete-path continuity checks |
| Simulation | Full-path playback, command highlighting, and step controls before execution |
| Spatial AI | Optional push-to-talk commands such as “move Part 3 right” or “place it in Bin A” |
| Offline development | Scene editing, simulation, and 146 automated tests without robot hardware |

Automatic background image classification is intentionally disabled. Physical objects use AprilTags or manually created virtual positions. The project does not require ROS 2.

This remains experimental robotics software. Physical accuracy depends on the individual robot, camera placement, tools, measurements, lighting, firmware, and calibration.

---

## Safety

Robots can pinch, strike, trap, drop, or launch objects. A successful simulation is not proof that a physical move is safe.

Before every physical run:

1. Clear people, cables, tools, and unmodeled obstacles from the workcell.
2. Verify the selected robot port and physically installed end effector.
3. Confirm the camera, permanent tags, part locations, and tool calibration.
4. Reduce the speed.
5. Run **Validate & Simulate** and inspect the complete path.
6. Keep the emergency stop or power disconnect within reach.
7. Never leave the robot unattended.

The software checks joint limits, coordinate bounds, firmware and host kinematics, path continuity, tool transforms, camera freshness, object movement, motion feedback, and explicit confirmation. It cannot verify the real table, cables, payload, suction seal, human presence, or every obstacle.

Do not loosen a failed safety check simply to make a move run.

---

## Hardware and compatibility

The platform has been developed and tested on macOS. Other operating systems have not been validated to the same level.

For physical use:

- Elephant Robotics myCobot 280 M5
- USB serial connection and supported robot firmware
- clear tabletop workspace and accessible emergency stop
- rigidly mounted external RGB camera
- ChArUco calibration board
- four 50 mm workspace AprilTags, IDs 0–3
- optional 30 mm object AprilTags, IDs 10–25
- ruler or calipers and a flat work surface
- adaptive gripper or Pump 2.0 suction accessory

On macOS, the camera selector intentionally excludes built-in FaceTime and Continuity/iPhone cameras and uses external cameras only.

---

## Understanding coordinates

Every saved location is measured from the center axis of the robot base. Imagine standing behind the robot and looking in the direction the robot faces:

| Direction | Coordinate change |
| --- | --- |
| Forward | X increases |
| Backward | X decreases |
| Left | Y increases |
| Right | Y decreases |
| Up | Z increases |

The table surface is normally `Z = 0`. Workcell positions are saved in meters. Robot firmware receives millimeters and degrees after conversion.

Four terms appear throughout the guide:

- **Part position:** center of the object.
- **Tag position:** center of the AprilTag taped to the object; it may be offset from the part center.
- **TCP:** the working point of the selected tool, such as the gripper center or suction-cup contact.
- **Flange:** the J6 mounting face where the tool attaches.

When the AI hears “move right,” it searches toward decreasing Y. “Move left” searches toward increasing Y. It never invents a coordinate; the server chooses and validates the actual destination.

---

## Connect the physical robot

### 1. Check firmware

Use Elephant Robotics’ supported Atom and base firmware for the myCobot 280 M5. Firmware mismatches can cause missing API functions, coordinate failures, or unstable serial communication.

### 2. Find the serial port

```bash
python3 web_server.py --list
```

Common examples:

- macOS: `/dev/cu.usbserial-...`
- Linux: `/dev/ttyUSB0` or `/dev/ttyACM0`
- Windows: `COM3`

### 3. Start with the port

```bash
python3 web_server.py \
  --host 127.0.0.1 \
  --web-port 8768 \
  --port /dev/cu.usbserial-XXXXXXXX \
  --baud 115200
```

You can also start without `--port` and select a detected port in the Robot inspector.

### 4. Confirm feedback

The dashboard should show:

- **Online**
- measured J1–J6 angles changing when the robot moves;
- the digital robot matching the physical robot;
- no serial read error in the status bar.

### 5. First motion

Use an unloaded, low-speed Home or short Joint Move:

1. Set speed override low.
2. Validate and simulate.
3. Inspect the path.
4. Request the physical run.
5. Confirm once.
6. Be ready to stop.

Do not begin with an outer-workspace pick.

---

## Dashboard tour

### Top bar

- Connection status
- Program workspace shortcut
- Global Stop

### Left scene tree

- Robot
- Parts
- Bins
- Points
- Programs

Select an item to edit it in the inspector.

### Center viewport

- Live digital robot
- Table and coordinate axes
- Camera visualization
- Parts and bins
- Taught points
- Planned paths
- Simulation

Drag operations modify virtual layout only where supported. A moved bin becomes `simulation_only` until its real position is confirmed.

### Right inspector

#### Inspector tab

Robot, tool, part, bin, point, and calibration settings.

#### Camera tab

External-camera selection, live frame, calibration wizard, tag visibility, and tracking status.

#### AI Assistant tab

OpenAI Realtime connection, push-to-talk, typed commands, assistant status, and results.

---

## Camera and AprilTag setup

### Why fiducials are used

The camera does not estimate robot coordinates from normalized pixels. Permanent workspace markers establish a measured pixel-to-robot table mapping on every valid frame. Object tags provide deterministic identity, XY position, and yaw.

### Tag allocation

| IDs | Purpose | Printed black-square size |
| --- | --- | --- |
| 0–3 | Permanent workspace references | 50 mm |
| 10–25 | Registered object tags | 30 mm |
| Other IDs | Not used by the current registry | — |

### Printable assets

Print at **100% / Actual Size**:

- [`static/calibration-assets/mycobot_charuco_board.pdf`](static/calibration-assets/mycobot_charuco_board.pdf)
- [`static/calibration-assets/mycobot_workspace_tags.pdf`](static/calibration-assets/mycobot_workspace_tags.pdf)
- [`static/calibration-assets/mycobot_object_tags_10_25.pdf`](static/calibration-assets/mycobot_object_tags_10_25.pdf)

Never use “Fit to page.” Confirm the black square of a workspace tag is exactly 50 mm and an object tag is exactly 30 mm.

### Physical workspace layout

1. Mount the external camera rigidly above the work surface.
2. Keep the complete usable table area visible.
3. Place workspace tags 0–3 near the usable corners.
4. Keep all tags flat.
5. Avoid glare, curved tape, torn edges, and heavy shadows.
6. Keep tags outside normal gripper contact areas.
7. Measure every workspace tag center from the projected J1 axis origin.
8. Enter measurements in meters using `+X forward` and `+Y left`.

Suggested arrangement:

```text
                  +X forward

        ID 0                      ID 1
     forward-left             forward-right

                usable workspace
                   robot

        ID 3                      ID 2
       rear-left                 rear-right

        +Y left  <----------->  -Y right
```

### Guided calibration

Select **Calibrate Camera** and complete the wizard.

#### Step 1: Print

- Print the ChArUco board.
- Print workspace tags 0–3.
- Optionally print object tags 10–25.
- Verify scale with a ruler.

#### Step 2: Camera and board photos

- Start the external camera.
- Capture at least 12 accepted ChArUco images.
- Cover the center and all four image regions.
- Use multiple distances.
- Include several tilted views.
- Keep the board sharp and sufficiently large.
- Use **Remove Last Photo** if the most recent sample is weak.

Practical intrinsic limits:

- intrinsic RMS no greater than 2.5 px;
- worst individual view no greater than 4 px.

If a capture says at least eight ChArUco corners are required:

- move the board closer;
- show more of the board;
- reduce glare;
- improve focus;
- keep the board inside the image;
- do not capture from an extreme angle.

#### Step 3: Enter workspace tag measurements

For IDs 0–3, enter:

- robot-frame X center;
- robot-frame Y center;
- marker yaw;
- 50 mm marker size.

Yaw zero uses the configured OpenCV corner order. Positive yaw is counterclockwise in the robot `+X/+Y` plane.

#### Step 4: Solve and lock

The system:

- undistorts the frame;
- detects the four reference tags;
- matches detected corners to measured robot-frame corners;
- estimates a RANSAC homography;
- reports conditioning, coverage, inliers, reprojection error, and per-marker error.

Lock the camera pose only after the marker geometry passes.

#### Step 5: Optional nine-point accuracy check

The nine-point check measures real XY accuracy across the table. It is recommended before precision physical picking but may be skipped in testing mode.

Practical targets:

- RMS XY error no greater than 10 mm;
- maximum XY error no greater than 20 mm;
- stationary spread no greater than 5 mm.

Skipping this step leaves an explicit warning. It does not make the coordinates more accurate.

### Understanding camera rejection messages

| Message | Meaning | Corrective action |
| --- | --- | --- |
| `not_all_reference_markers_visible` | A required workspace tag is missing | Reframe, remove occlusion, improve lighting |
| `homography_poorly_conditioned` | Robot-frame tag geometry cannot support a stable mapping | Check signs, duplicate coordinates, corner order, and layout |
| `reprojection_error_excessive` | One homography does not explain all measured corners | Recheck intrinsics, tag measurements, flatness, scale, and yaw |
| `marker_inliers_insufficient` | One or more tags disagree with the others | Inspect per-marker diagnostics and remeasure the failing tag |
| `camera_moved_reaccept_required` | Current tag geometry differs from the locked baseline | Return the camera or explicitly accept and verify the new pose |
| `intrinsics_missing` | Lens calibration has not been solved | Complete ChArUco capture and solve |
| `outside_workspace_bounds` | Projected object position is outside configured limits | Correct calibration or move the object into the usable area |

Detection alone is not enough. The system rejects a visible tag when its pixels do not agree with the measured physical geometry.

### Register a tagged part

1. Select **+** beside Parts.
2. Choose **Track with AprilTag**.
3. Start the calibrated camera.
4. Select a visible tag polygon or its explicit selectable tag entry.
5. Enter name, shape, color, length, width, height, and graspability.
6. Enter the top-face tag offset if it is not centered.
7. Enter the tag-to-object yaw alignment.
8. Review the computed pose.
9. Save.

Tags 0–3 cannot be assigned to parts. An already-bound object tag requires explicit reassignment.

### Object-tag placement

- Tape the tag flat to a planar top surface.
- Use the printed 30 mm black square exactly.
- Align it with the object length axis when possible.
- If it is offset, measure from the object center in object-local coordinates.
- Enter the object’s real height; monocular tracking does not infer it.

### Tracking behavior

- Tracking updates locally around every 80 ms.
- A three-frame robust filter stabilizes position and wrapped yaw.
- Identity comes directly from the tag ID.
- A part disappears from the live scene after approximately one second without a valid observation.
- Its registered definition remains and returns with the same identity when visible again.
- No OpenAI request is made merely because the camera is running.

---

## Parts, bins, and points

### Parts

#### Tagged parts

A tagged part stores:

- stable part ID;
- AprilTag ID;
- name;
- shape;
- color;
- dimensions;
- graspability;
- tag size;
- tag offset;
- yaw offset;
- tool-specific pickup profiles.

The live pose exists only while the tag is valid and recent.

#### Virtual-only parts

Use virtual-only parts for:

- simulation;
- layout planning;
- objects that do not need camera tracking.

Virtual coordinates are not camera-verified physical coordinates.

#### Pickup setup

Each part can override:

- object-local X/Y/Z contact offset;
- adaptive jaw yaw;
- suction contact offset;
- suction preload.

Tag-placement offset and pickup offset are separate. Moving the tag definition does not intentionally move the grasp point.

### Bins

A bin has dimensions, floor position, color, and a verification state.

| State | Meaning |
| --- | --- |
| `operator_verified` | The saved digital position is treated as matching the real bin |
| `simulation_only` | The bin was moved virtually and cannot authorize physical placement |

After moving a bin in the digital scene:

1. Move the real bin to the displayed location.
2. Measure or verify it.
3. Select **Confirm Physical Position** in the inspector.

### Taught points

A taught point stores:

- stable ID and name;
- measured six-joint configuration;
- measured firmware flange coordinates;
- derived TCP pose;
- active end effector;
- tool-calibration fingerprint;
- optional support-surface Z;
- allowed use as a waypoint or object destination;
- timestamps.

It does not store “inverse kinematics.” It stores the measured robot state and uses that state as a preferred seed while the normal validator independently checks the target.

### Create a taught point

1. Connect the robot.
2. Select the physically installed tool.
3. Open Points and choose **Create Point**.
4. Release joints for hand-guiding, or jog to the desired pose.
5. Stop and let the robot settle.
6. Select **Capture Current Point**.
7. Verify angles, flange pose, TCP pose, and tool.
8. Name and save the point.

Recapture after changing tools or changing the tool calibration.

---

## End effectors

### Adaptive gripper

The adaptive-gripper planner:

- uses a centered narrow-side pinch by default;
- keeps the jaw center aligned with the object center in XY;
- chooses jaw yaw from the object footprint;
- limits fallback tilt to 10°;
- models a 21 mm usable finger contact section;
- maintains configurable table clearance;
- supports a tool-local contact correction;
- supports per-part pickup offsets.

### Pump 2.0 suction tool

Default installed geometry:

| Measurement | Value |
| --- | ---: |
| Flange to cup start | 50 mm |
| Cup free extension | 22 mm |
| Flange to contact | 72 mm |
| Installed cup diameter | 22 mm |
| Nominal pump box | 72 × 52 × 37 mm |
| Nominal wrist head | 63 × 24.5 × 26.7 mm |
| Published total accessory mass | 180 g |
| Published rated suction payload | 150 g |

No physical center of mass is claimed because the source URDF does not publish inertial data. The base pump is excluded from moving-arm payload calculations.

The suction planner:

- targets the object top face;
- defaults to the top-face center;
- uses 2 mm compliant preload;
- verifies that the complete cup fits on the object footprint;
- holds one orientation through approach, contact, and lift;
- treats round-cup yaw as symmetric;
- prevents unnecessary J6 motion;
- does not reuse adaptive side-pinch height formulas.

### Pump 2.0 wiring

Default base IO:

- Pin 5: pump
- Pin 2: release valve
- Both active-low

Suction on:

```text
2 → 1   close release valve
5 → 0   start pump
```

Suction off:

```text
5 → 1   stop pump
2 → 0   open release valve
wait 1 second
2 → 1   close release valve
```

The accessory also requires its correct pump power connection. The base IO pins provide control signals; wiring and external accessory power must follow the accessory documentation.

Environment overrides:

```bash
export MYCOBOT_SUCTION_PROFILE=pump_v2
export MYCOBOT_SUCTION_ON="2:1,5:0"
export MYCOBOT_SUCTION_OFF="5:1,2:0,sleep:1.0,2:1"
```

Available named profiles:

- `pump_v2`
- `legacy_split_valve`
- `both_low`
- `inverted_split`

Use the suction diagnostic panel at low risk before attempting a pick.

---

## Tool contact calibration

Do not falsify object dimensions or camera coordinates to compensate for a tool miss.

Use **Tool Contact Calibration** when the complete tool is systematically:

- left or right;
- forward or back;
- high or low.

Enter the observed physical miss in millimeters. The software converts it into a tool-local XYZ correction and applies it consistently to:

- planning;
- flange/TCP conversion;
- host IK;
- firmware preview;
- path visualization;
- execution diagnostics.

Calibration is stored separately for each end effector.

### Recommended procedure

1. Use a clearly marked, large, centered test target.
2. Reduce speed.
3. Plan a simple vertical approach.
4. Stop before contact if necessary.
5. Measure the miss at the intended TCP.
6. Enter the observed miss.
7. Validate and simulate again.
8. Repeat until the systematic error is removed.

Use per-part pickup offsets only when you intentionally want a non-centered contact point.

---

## Creating programs

Open a saved program from the Programs tree, select **+** to create one, or use the top-bar Program button.

### Workspace layout

- Left: hierarchical command tree
- Center: live workcell and validated path
- Right: selected-command settings
- Header: repeat count, Save, Delete, Close
- Footer: connection, tool, speed override, status, Stop, Validate & Simulate, Run

### Command palette

#### Motion

| Command | Behavior |
| --- | --- |
| Joint Move | Replays a captured six-joint configuration with joint interpolation |
| Linear Move | Replays a captured TCP pose through firmware linear coordinate motion |
| Home | Moves to the validated home joint configuration |

#### Smart skills

| Command | Behavior |
| --- | --- |
| Pick Part | Expands into tool-specific approach, contact, acquire, and lift states |
| Place Object | Expands into transfer, lower, release, and retreat states |

#### Tool

| Command | Adaptive gripper | Suction |
| --- | --- | --- |
| Acquire | Close / grip | Suction on |
| Release | Open / release | Suction off and vent |

#### Utility

| Command | Behavior |
| --- | --- |
| Wait | Pauses for a bounded duration |

### Edit command tree

Commands can be:

- inserted at a selected position;
- reordered;
- duplicated;
- renamed;
- disabled;
- deleted.

Programs can repeat 1–20 times. All iterations and transitions are expanded and validated before approval.

### Embedded and linked waypoints

Every motion node owns an embedded waypoint by default.

You may:

- keep it embedded in the program;
- save a copy to the global Points library;
- link the motion node to a shared point;
- detach a linked point into an independent embedded copy.

Deleting a global taught point invalidates only steps that are genuinely linked to it. Embedded waypoints remain inside the program.

### Edit Point

#### Joint Jog

- J1–J6 hold `−/+`
- 0.5°, 1°, or 5° increments
- speed 1–30

#### TCP Jog

- X/Y/Z in the robot-base frame
- Rx/Ry/Rz orientation
- 1, 5, or 10 mm translation increments
- 0.5°, 1°, or 5° rotation increments

#### Hand Guide

- release joints;
- position the robot manually;
- recapture after settling.

J6 jogging is disabled for the symmetric suction tool.

### Jog safety

- only one active jog session;
- heartbeat watchdog;
- joint-limit stopping;
- maximum hold duration;
- stop on pointer release;
- stop on browser blur;
- stop when the dialog closes;
- stop on connection loss;
- any jog invalidates the existing preview.

### Save and close behavior

- Save is in the programmer header.
- Delete is available after opening a program and requires confirmation.
- Closing with unsaved changes requires Save, Discard, or Cancel.
- The main Programs list opens programs but does not contain duplicate delete buttons.

---

## Simulation and physical execution

### Validation layers

1. Program schema and command completeness
2. Part, bin, and taught-point references
3. Camera freshness and bin verification
4. Coordinate bounds
5. Tool and calibration fingerprint
6. Firmware IK result
7. Firmware FK round trip
8. Independent host FK residual
9. Independent host IK reachability
10. Joint limits
11. Adjacent joint continuity
12. Every subdivided waypoint
13. Complete repeated-program transitions

Accuracy targets are 3 mm and 3°. A larger 15 mm / 5° host-model envelope is a hard disagreement gate, not an accuracy claim.

### Orientation selection

- Strict top-down is tried first.
- If necessary, fixed tilts of 2°, 4°, 6°, 8°, and 10° are tested.
- One orientation must pass the complete associated segment.
- Orientation remains fixed during a vertical descend or lift.
- Targets are not silently shifted to make IK pass.

### Transfer planning

Long cross-table transfers are subdivided only when necessary for:

- joint continuity;
- bearing changes;
- avoiding the modeled robot-base exclusion.

The planner prefers fewer direct waypoints when the complete transition validates.

### Simulation controls

- Play
- Pause
- Previous
- Next
- Reset
- From Here

“From Here” is simulation only. Physical partial execution is intentionally unavailable.

### Run states

| Button text | Meaning |
| --- | --- |
| Validate & Simulate First | No valid complete preview exists |
| Blocked — View Issue | A preview exists but a specific preflight gate blocks execution |
| Run Complete Program | Fresh validated preview is ready |
| Running | One execution request is active |

### Physical execution

On a valid run:

1. Visual simulation stops.
2. Exactly one execution request is sent.
3. Speed override is applied server-side.
4. Preflight runs again against current robot angles.
5. Enabled commands execute in order.
6. Each motion is verified.
7. Runtime progress highlights the active source node.
8. The preview is consumed after the attempt.

### Controller error codes

Common firmware codes:

| Code | Meaning |
| ---: | --- |
| 0 | No reported controller error |
| 1–6 | Corresponding joint limit error |
| 16–19 | Collision-protection error |
| 32 | No firmware IK solution |
| 33–34 | No adjacent solution for linear motion |

The UI distinguishes host IK failure, firmware IK rejection, controller refusal, endpoint miss, disconnection, and transport failure.

---

## Spatial AI assistant

The assistant is optional. Robot control, camera localization, simulation, and manual programming work without it.

### Configure

```bash
cp api_keys.env.example api_keys.env
```

Edit `api_keys.env`:

```dotenv
OPENAI_API_KEY=sk-your-key-here
OPENAI_REALTIME_MODEL=gpt-realtime-2.1
OPENAI_REALTIME_VOICE=marin
```

Never commit `api_keys.env` or `.env`.

### Connect

1. Open the AI Assistant tab.
2. Select **Connect**.
3. Hold **Hold to Talk**.
4. Speak.
5. Release to send.

The microphone is not an open always-listening channel. Very short presses are ignored to avoid invalid audio commits.

### Example commands

- “What parts are visible?”
- “Move Part 3 to the right.”
- “Put Part 3 in Bin A.”
- “Move Bin A right and simulate placing Part 3 in it.”
- “Go to Inspection Point.”
- “Pick Part 3 and place it at Drop Point.”
- “Plan a home move.”
- “Run that.”
- “Yes.”
- “Stop.”

### Division of responsibility

GPT may choose:

- the referenced object;
- a named region;
- a bin;
- a taught point;
- the intended high-level program.

Deterministic server code chooses:

- exact robot-frame coordinates;
- workspace margins;
- occupancy clearance;
- base exclusion;
- target freshness;
- IK candidates;
- complete path;
- whether execution is allowed.

The model may not invent coordinates, hidden-object locations, embedded waypoints, joint values, or confirmation tokens.

### Physical confirmation

A voice request to run stages the latest preview and asks one short yes/no question. Only a clear confirmation within the pending-run window may start physical execution.

### Camera classification

There is no background semantic classification loop. An explicit assistant action may suggest a label and shape for one visible tagged part. Applying the suggestion requires confirmation and cannot alter geometry, dimensions, tag binding, or coordinates.

---

## Data, backups, and privacy

### Workcell data

`data/workcell.json` stores:

- virtual parts;
- registered tagged-part definitions;
- bins;
- taught points;
- programs;
- camera selection;
- camera intrinsics;
- workspace-marker measurements;
- baseline homography;
- verification results;
- active end effector;
- tool calibration;
- coordinate-planner configuration.

Live tag poses remain in memory and are not rewritten to disk every frame.

### Back up

Stop the server, then:

```bash
cp data/workcell.json data/workcell.backup.json
```

Restore only while the server is stopped:

```bash
cp data/workcell.backup.json data/workcell.json
```

### Reset

Back up first. Removing `data/workcell.json` creates an empty default workcell on the next start.

### Move to another Mac

Install the platform using the [Quick start](#quick-start). Copy `data/workcell.json` only when the second computer will control the same physical robot, camera, tags, table, and tools.

Start with a new workcell and recalibrate when the camera, resolution, camera mount, workspace tags, table, robot-base position, or installed tool changed.

### Privacy

`data/workcell.json` may contain:

- external-camera unique identifiers;
- physical workspace measurements;
- camera calibration matrices;
- object names;
- taught robot poses;
- program definitions.

Review or replace it before publishing a fork.

The live file is ignored by Git. [`data/workcell.example.json`](data/workcell.example.json) is the sanitized public example.

---

## Configuration

### API key file

Preferred optional assistant configuration:

```text
api_keys.env
```

Supported variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `OPENAI_API_KEY` | none | Enables Realtime assistant |
| `OPENAI_REALTIME_MODEL` | `gpt-realtime-2.1` | Realtime model override |
| `OPENAI_REALTIME_VOICE` | `marin` | Voice override |

### Suction variables

| Variable | Default | Purpose |
| --- | --- | --- |
| `MYCOBOT_SUCTION_PROFILE` | `pump_v2` | Named wiring sequence |
| `MYCOBOT_SUCTION_ON` | profile sequence | Custom comma-separated actions |
| `MYCOBOT_SUCTION_OFF` | profile sequence | Custom comma-separated actions |

### Restart-script variables

`restart_server.sh` is a portable developer convenience script. It starts disconnected
with `python3` by default; provide a robot port or interpreter only when needed:

| Variable | Purpose |
| --- | --- |
| `ROBOT_PORT` | Serial device |
| `WEB_PORT` | Dashboard port |
| `BAUD` | Serial baud |
| `PYTHON_BIN` | Python executable |
| `FOREGROUND=1` | Run attached to the terminal |

Example:

```bash
ROBOT_PORT=/dev/cu.usbserial-XXXX \
WEB_PORT=8768 \
BAUD=115200 \
PYTHON_BIN="$PWD/.venv/bin/python3" \
FOREGROUND=1 \
./restart_server.sh
```

Running the script without `ROBOT_PORT` is safe for offline dashboard use. Select a
detected robot port later in the Robot inspector.

---

## Command line

```text
python3 web_server.py [options]
```

| Option | Default | Description |
| --- | --- | --- |
| `--port` | none | Robot serial port |
| `--baud` | `115200` | Robot baud rate |
| `--timeout` | `0.8` | Serial response timeout in seconds |
| `--host` | `127.0.0.1` | Loopback-only HTTP bind address |
| `--web-port` | `8765` | HTTP port |
| `--list` | false | List serial ports and exit |

Examples:

```bash
# Offline dashboard
python3 web_server.py --web-port 8768

# Physical robot
python3 web_server.py \
  --port /dev/cu.usbserial-XXXXXXXX \
  --baud 115200 \
  --web-port 8768

# List ports
python3 web_server.py --list
```

CobotWorkcell refuses non-loopback addresses such as `0.0.0.0` or a LAN IP.
The API contains physical-control endpoints and does not provide remote
authentication, so the public release is intentionally accessible only from
the computer running the server.

---

## HTTP API

The browser uses a local JSON API served by `web_server.py`.

### General behavior

- JSON responses use `ok: true/false` where applicable.
- Invalid JSON returns HTTP 400.
- Unknown API endpoints return HTTP 404.
- Unexpected server errors return HTTP 500.
- Physical execution may return a stale-preview conflict.
- Camera frame endpoints return JPEG or MJPEG rather than JSON.

### Read-only GET endpoints

| Endpoint | Purpose |
| --- | --- |
| `GET /api/status` | Robot connection, execution, and progress |
| `GET /api/angles` | Current six joint angles |
| `GET /api/coords` | Current firmware flange coordinates |
| `GET /api/kinematics/frame-snapshot` | Read-only flange/TCP comparison |
| `GET /api/ports` | Detected serial ports |
| `GET /api/scene` | Complete persistent and live scene snapshot |
| `GET /api/realtime/status` | Assistant configuration availability |
| `GET /api/camera/status` | Camera runtime and saved calibration |
| `GET /api/camera/localization/status` | Current localization quality |
| `GET /api/camera/tags/visible` | Visible tag polygons and binding state |
| `GET /api/camera/tag-tracks?since=N` | Incremental live tagged-part pose changes |
| `GET /api/camera/debug-frame` | Annotated localization JPEG |
| `GET /api/camera/devices` | Available camera devices |
| `GET /api/camera/frame` | Latest camera JPEG |
| `GET /api/camera/stream` | MJPEG camera stream |

### Scene and configuration POST endpoints

| Endpoint | Purpose |
| --- | --- |
| `POST /api/config` | Select robot port and baud |
| `POST /api/scene/part` | Create or update a virtual part |
| `POST /api/scene/part/tag-binding` | Create, bind, update, or explicitly reassign a tagged part |
| `POST /api/scene/part/tag-unbind` | Convert a tagged part to virtual at its last valid pose |
| `POST /api/scene/part/delete` | Delete a part and its registration |
| `POST /api/scene/bin` | Create or update a bin |
| `POST /api/scene/bin/confirm-position` | Mark the real bin position operator-verified |
| `POST /api/scene/bin/delete` | Delete a bin |
| `POST /api/scene/point` | Save a taught point |
| `POST /api/scene/point/delete` | Delete a taught point |
| `POST /api/scene/end-effector` | Select adaptive or suction tool |
| `POST /api/scene/coordinate-planner` | Update tool/planner calibration |
| `POST /api/scene/clear` | Clear scene parts |
| `POST /api/spatial/resolve` | Resolve a spatial destination deterministically |

### Program endpoints

| Endpoint | Purpose |
| --- | --- |
| `POST /api/programs/save` | Save a program |
| `POST /api/programs/delete` | Delete a saved program |
| `POST /api/program/plan` | Compile and validate a program |
| `POST /api/program/release-preview` | Release preview reservations |
| `POST /api/program/execute` | Execute an already validated plan after confirmation |
| `POST /api/pick/simulate` | Build a pick preview |

### Robot-control endpoints

> **Warning:** These endpoints can affect physical hardware when connected.

| Endpoint | Purpose |
| --- | --- |
| `POST /api/send-angles` | Send six joint targets |
| `POST /api/robot/jog/start` | Start guarded hold-to-jog |
| `POST /api/robot/jog/heartbeat` | Keep the active jog session alive |
| `POST /api/robot/jog/step` | Perform a bounded joint/TCP increment |
| `POST /api/robot/jog/stop` | Stop active jogging |
| `POST /api/robot/points/capture` | Capture measured angles/flange/TCP |
| `POST /api/robot/capture-tool-orientation` | Save current firmware RPY |
| `POST /api/command/<action>` | Power, stop, home, gripper, suction, and diagnostic actions |

### Camera calibration endpoints

| Endpoint | Purpose |
| --- | --- |
| `POST /api/camera/config` | Save selected camera configuration |
| `POST /api/camera/start` | Start camera and localization |
| `POST /api/camera/stop` | Stop camera and localization |
| `POST /api/camera/calibration/charuco/clear` | Clear captured intrinsic samples |
| `POST /api/camera/calibration/charuco/remove-last` | Remove latest sample |
| `POST /api/camera/calibration/charuco/capture` | Capture current ChArUco frame |
| `POST /api/camera/calibration/charuco/solve` | Solve and save intrinsics |
| `POST /api/camera/calibration/workspace` | Save workspace marker map |
| `POST /api/camera/calibration/verify` | Process one current localization frame |
| `POST /api/camera/calibration/verification-report` | Save nine-point report |
| `POST /api/camera/calibration/verification-skip` | Enter explicit testing bypass |
| `POST /api/camera/calibration/accept-pose` | Lock current valid camera pose |

### Realtime endpoints

| Endpoint | Purpose |
| --- | --- |
| `POST /api/realtime/session` | Create a short-lived Realtime WebRTC session |
| `POST /api/realtime/tool` | Execute an allowed deterministic assistant tool |

### Safe API examples

Read robot status:

```bash
curl http://127.0.0.1:8768/api/status
```

Read scene:

```bash
curl http://127.0.0.1:8768/api/scene
```

List camera devices:

```bash
curl http://127.0.0.1:8768/api/camera/devices
```

Do not script physical execution until you understand the preview, confirmation, freshness, and motion-verification contracts in `web_server.py`.

---

## Architecture

The browser communicates with the local Python server. The server owns the workcell, planning, validation, camera, and robot-driver state. `data/workcell.json` stores persistent settings, while fast camera-tag updates remain in memory. The optional AI assistant requests high-level actions through the same validated server APIs used by the dashboard.

### Motion authority

- The host creates and validates targets.
- Firmware remains responsible for physical coordinate IK and execution.
- Host kinematics independently validates the firmware result.
- The UI cannot authorize motion without the server’s physical confirmation gate.

### Camera data flow

1. Read and undistort the external-camera frame.
2. Locate workspace tags 0–3 and calculate the table mapping.
3. Reject the frame if its quality checks fail.
4. Locate object tags 10–25.
5. Calculate and filter each registered object pose.
6. Update the live scene without rewriting the workcell file every frame.

### Planning data flow

1. Resolve every referenced part, bin, and point.
2. Expand Pick, Place, tool actions, waits, and repeats.
3. Apply the active tool transform.
4. Validate firmware and independent host kinematics.
5. Check limits, continuity, and every path segment.
6. Create the immutable simulation preview.
7. Recheck current robot and camera state before execution.
8. Dispatch and verify each physical command.

---

## Repository structure

| Path | Responsibility |
| --- | --- |
| `web_server.py` | HTTP API, robot connection, preview validation, execution gates, Realtime tools |
| `workcell.py` | Persistent scene, programs, objects, bins, points, spatial resolution, grasp planning |
| `mycobot_driver.py` | `pymycobot` serial and firmware adapter |
| `mycobot_kinematics.py` | Flange/TCP transforms, FK, host IK, vertical candidates |
| `fiducial_localization.py` | ChArUco, AprilTags, homography, tracking, verification |
| `camera_service.py` | External-camera discovery, capture, JPEG stream |
| `generate_calibration_pdfs.py` | Printable calibration-board generation |
| `generate_fiducial_assets.py` | Tag asset generation |
| `static/index.html` | Dashboard markup |
| `static/styles.css` | Dashboard and programmer layout |
| `static/js/main.js` | Polling and application boot |
| `static/js/store.js` | Shared client state |
| `static/js/ui.js` | Inspector, wizard, programmer, jogging |
| `static/js/viewport.js` | Three.js scene, digital twin, paths, simulation |
| `static/js/realtime.js` | Push-to-talk Realtime assistant |
| `static/vendor/` | Robot, tool, Three.js, and attributed CAD assets |
| `data/workcell.json` | Machine-specific persistent workcell |
| `tests/` | Offline regression suite |
| `.github/` | CI, maintenance, templates, and dependency monitoring |

---

## Testing

### Run everything

```bash
python3 scripts/run_offline_checks.py
```

The runner creates a disposable Python 3 virtual environment, installs the
declared dependencies, then compiles the source, runs `git diff --check`, and
runs the offline suite. It does not access robot or camera hardware.

The current suite contains 146 offline tests.

### Test areas

- camera enumeration and external-only policy;
- camera status and stale-frame handling;
- ChArUco capture and intrinsic calibration;
- homography recovery and conditioning;
- marker occlusion and unknown IDs;
- per-marker inlier diagnostics;
- moved-camera rejection;
- elevated object-tag pose recovery;
- tag yaw, offsets, filtering, disappearance, and reappearance;
- saved-workcell migration;
- coordinate frame transforms;
- flange/TCP round trips;
- joint limits and angle wrapping;
- host FK and IK;
- reachable and unreachable poses;
- tilted top-down candidates;
- suction and adaptive geometry;
- pick height and table clearance;
- path continuity and transfer subdivision;
- stale object snapshots;
- spatial directions and placement search;
- taught-point capture and tool matching;
- programmer migration and command compilation;
- embedded and linked waypoints;
- program deletion and physical-run contracts;
- joint and TCP jogging safeguards;
- Realtime tool flow and push-to-talk ordering;
- browser module-state consistency;
- layout containment and renderer resizing;
- pump IO sequencing;
- motion polling and failure diagnostics.

### Test safety

The automated suite uses fake or preview robot objects and does not command physical motion.

Do not add a test that opens a real serial port, starts a physical camera, changes IO, or moves hardware unless it is explicitly separated and operator-gated.

### Before submitting a change

1. Run the complete suite.
2. Run `git diff --check`.
3. Start the dashboard disconnected.
4. Check browser console errors.
5. Inspect the programmer at common laptop widths.
6. Verify that tests did not modify `data/workcell.json`.
7. Describe any untested physical behavior in the pull request.

---

## Troubleshooting

### `Missing dependency: pyserial` or `pymycobot`

Activate the project environment:

```bash
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

### `cv2` has no attribute `aruco`

Install the contrib build:

```bash
python3 -m pip uninstall -y opencv-python
python3 -m pip install "opencv-contrib-python>=4.8"
```

### Server cannot bind the port

Example:

```text
Cannot listen on 127.0.0.1:8768
```

Check:

```bash
lsof -nP -iTCP:8768 -sTCP:LISTEN
```

Stop the old process or select another `--web-port`.

### Dashboard changes do not appear

- Reload the page.
- Perform a hard refresh if a static JavaScript or CSS file was cached.
- Restart `web_server.py` after Python changes.
- Frontend-only changes normally need only a browser reload.

### Programmer stretches horizontally during simulation

The current layout constrains long runtime diagnostics and resizes the Three.js renderer with its container. Hard-refresh to ensure the newest versioned stylesheet and JavaScript are loaded.

### No robot ports appear

- Verify the USB cable supports data.
- Reconnect the robot.
- Check system serial devices.
- Close other programs using the port.
- Run `python3 web_server.py --list`.
- On Linux, verify device permissions.

### Robot shows Online but does not match the real pose

- Confirm angle polling is succeeding.
- Verify the correct serial port and robot model.
- Confirm the firmware version.
- Check for transient read errors.
- Restart the server after driver changes.
- Do not use a different application to control the same serial port simultaneously.

### Robot freezes or refuses a coordinate move

Inspect the exact error:

- `host_ik_unreachable`: independent host model cannot solve the pose;
- `firmware_ik_rejected`: firmware result failed validation;
- `controller_ik_no_solution`: controller code 32;
- `stopped_outside_tolerance`: robot stopped but endpoint verification failed;
- `coordinate_target_missed`: endpoint remained outside the physical envelope.

A huge endpoint miss with no measurable motion usually means the controller refused the pose, not that polling was too strict.

### `No vertical or <=10 deg tilted orientation passed`

The complete approach/contact/lift segment did not find one independently valid fixed orientation.

Check:

- part coordinates;
- active tool;
- TCP calibration;
- object height;
- target radius;
- joint starting state;
- whether the desired top-down pose exceeds the robot’s real wrist workspace.

Do not assume a target is reachable merely because its XYZ is inside a rectangular bound.

### `joint discontinuity`

The endpoints may be reachable on different IK branches. The planner will add only necessary continuity waypoints. If still blocked:

- start from a less folded pose;
- add an intentional taught waypoint;
- reduce cross-base travel;
- check unnecessary J6 rotation;
- verify the active suction J6 lock.

### Joint Move says an embedded taught point no longer exists

This was a software defect in older builds. Embedded waypoints are step-owned and do not require a global Points entry. Update to the current source and revalidate.

### Camera list is empty on macOS

The project intentionally excludes:

- FaceTime/built-in cameras;
- iPhone Continuity Camera;
- iPad cameras;
- Desk View.

Connect an external USB camera, grant camera permission to the terminal/Python process, and select **Find Cameras**.

### Selecting an external camera opens FaceTime

Update to the current camera service. It maps the selected external camera by stable AVFoundation identity and mirrors OpenCV’s sorted device order. It refuses to silently open a different device.

### Camera status says `Failed to fetch`

- Confirm the Python server is running.
- Reload the dashboard.
- Check the terminal for a server exception.
- Open `/api/camera/status` directly.
- Restart after Python changes.

### ChArUco capture is rejected

The board must provide at least eight recognized ChArUco corners. Move it closer, improve focus and lighting, show more squares, and avoid extreme tilt.

### Workspace tags are visible but rejected

Visibility proves only that the tag was decoded. Localization also checks whether detected corners agree with:

- camera intrinsics;
- tag center;
- tag size;
- tag yaw;
- corner order;
- the other markers.

Use the per-marker RMS, maximum, and inlier count to identify the bad measurement.

### Do I need the nine-point test every time?

No. Lens calibration is tied to the camera and resolution. A moved camera requires accepting and verifying its new workspace pose, but the nine-point procedure may be skipped in testing mode. Precision claims should not be made until it passes.

### A part tag cannot be selected

- IDs 0–3 are reserved.
- Only IDs 10–25 are selectable object tags.
- Ensure the tag is visible in the current camera frame.
- Check the explicit selectable-tag list if polygon hit-testing is difficult.
- Resolve `camera_moved_reaccept_required`.
- Reassignment of an already-bound tag requires confirmation.

### Tracked part disappears

That is expected after approximately one second without a valid tag observation. Remove occlusion or restore localization. The registry definition is retained.

### Suction does not activate

- Confirm the accessory has pump power.
- Confirm it is wired to base IO, not Atom tool-head IO.
- Confirm Pump 2.0 pin mapping.
- Use the diagnostic controls for pump pin 5 and release pin 2.
- Try only the correct named legacy profile if your harness is known to differ.
- Inspect the terminal for `set_basic_output` availability.

### Suction cannot pick the object

- Confirm the cup is fully inside the top footprint.
- Confirm the surface is flat and nonporous.
- Check the 72 mm TCP.
- Calibrate high/low miss.
- Confirm the object mass is below both robot and suction payload limits.
- Increase pump settle time only after verifying wiring and contact.

### Pick is consistently left/right/high/low

Use Tool Contact Calibration. Do not change object dimensions or tag coordinates to hide a systematic tool transform error.

### AI says planning but nothing appears

- Check the Realtime status.
- Verify no earlier response or tool call is still active.
- Look for a plan in the programmer workspace.
- Confirm the referenced part is visible.
- Confirm a bin is physically verified.
- Try a taught point or named region.
- Inspect the exact deterministic planning error in the program output.

### Push-to-talk reports buffer too small

Hold the button for more than 200 ms before releasing. The client clears prior audio, commits only a sufficiently long buffer, and prevents overlapping responses.

### Physical run button appears blocked

Click the blocked button or inspect the footer for the specific cause:

- stale tracked part;
- hidden part;
- simulation-only bin;
- tool mismatch;
- missing waypoint;
- invalid coordinate preview;
- disconnected robot;
- modified linked point;
- changed tool calibration.

---

## Repository automation

The repository includes a low-touch automation model with a human merge gate:

1. **Continuous integration** runs the complete offline test suite on pushes and pull requests.
2. **Scheduled health checks** rerun the suite and open or update a GitHub issue when the default branch fails.
3. **Dependabot** proposes dependency and GitHub Actions updates.
4. **Issue intake** automatically labels bug and feature reports `needs-triage`.
5. **Weekday Codex maintenance** classifies new reports, autonomously promotes only bounded low-risk software work, and may prepare an isolated repair branch and draft pull request.
6. **Maintainer review** remains mandatory before merge, hardware validation, releases, or safety-policy changes.

| Automation | File or location | Behavior |
| --- | --- | --- |
| Pull-request and main-branch CI | [`.github/workflows/ci.yml`](.github/workflows/ci.yml) | Python 3.10/3.12 compile, tests, and tracked-secret check |
| Scheduled health monitor | [`.github/workflows/health-monitor.yml`](.github/workflows/health-monitor.yml) | Weekly test run, diagnostic artifact, and deduplicated failure issue |
| Dependency updates | [`.github/dependabot.yml`](.github/dependabot.yml) | Weekly grouped pip and GitHub Actions pull requests |
| AI maintainer | Codex desktop automation | Weekday full-queue triage and up to three independent reviewable fix branches per run |
| Agent safety rules | [`AGENTS.md`](AGENTS.md) | No hardware, secrets, live workcell edits, direct-main pushes, self-merges, or releases |

### Issue intake and classification

Bug reports enter with `bug` and `needs-triage`; feature requests enter with
`enhancement` and `needs-triage`. Issue content is untrusted input. Automation
does not execute pasted commands or treat reporter instructions as repository
authority.

During its scheduled run, Codex may:

- identify duplicates and request missing reproduction details;
- add an appropriate category or `needs-hardware-validation`;
- promote a clear, bounded, offline-reproducible software issue to
  `codex-ready`;
- map ready issues to existing branches and pull requests so work is not
  duplicated;
- process up to three independent eligible issues, with one isolated branch,
  test run, and draft pull request per issue.

Issues that overlap in files, behavior, or validation are not combined.
Automation processes the highest-priority one and leaves the others queued with
a conflict note.

Codex must leave an issue for maintainer review when it involves security,
dependencies, GitHub workflows or permissions, releases, destructive data
migration, physical hardware, or changes to kinematic, collision, calibration,
freshness, confirmation, or motion-verification safety.

The maintainer may apply `codex-ready` manually to authorize other bounded
software work. Neither automatic nor manual authorization permits Codex to
merge the resulting pull request.

### Automation safety policy

Automation must never:

- connect to a robot serial port;
- start a physical camera;
- modify pump or gripper IO;
- command joint or Cartesian motion;
- overwrite `data/workcell.json`;
- expose API keys or device identifiers;
- push directly to `main`;
- merge its own pull request;
- publish a release without maintainer approval.

### Bot-created changes

An automated fix is acceptable only when it:

1. references a reproducible issue;
2. uses an isolated branch or worktree;
3. includes a regression test;
4. passes the complete offline suite;
5. reports limitations and physical validation still required;
6. opens a pull request for human review.

### Release recommendation

A maintenance bot may recommend:

- patch release: compatible bug fixes;
- minor release: backward-compatible features;
- major release: incompatible API or data changes.

It must not create the release automatically. The maintainer reviews the changelog, migration impact, workcell-data safety, and physical validation before tagging.

### GitHub repository settings

After publishing:

1. Enable GitHub Actions.
2. Give workflows read-only contents permission by default.
3. Allow issue creation only where needed.
4. Protect `main`.
5. Require the offline CI check.
6. Require pull-request review.
7. Disable force pushes and branch deletion.
8. Enable Dependabot security updates.
9. Enable secret scanning.
10. Consider GitHub Discussions for community support.

---

## Public release checklist

### 1. Choose an open-source license

Complete: CobotWorkcell's original source code and documentation are licensed
under the [Apache License 2.0](LICENSE). Vendored third-party components retain
their respective licenses and attribution requirements.

### 2. Remove secrets

Verify:

```bash
git ls-files | grep -E '(^|/)(\\.env|api_keys\\.env)$'
```

The command should return nothing.

Never publish:

- `OPENAI_API_KEY`;
- personal access tokens;
- serial credentials;
- private URLs;
- logs containing secrets.

If a secret was ever committed, deleting it from the latest file is not enough. Revoke it and clean Git history.

### 3. Verify machine-specific workcell data

The live `data/workcell.json` is ignored and must remain untracked. The repository includes only [`data/workcell.example.json`](data/workcell.example.json).

Before publishing, verify that neither the current tree nor Git history contains a real workcell file.

### 4. Review restart defaults

Complete: `restart_server.sh` now discovers the repository and Python environment portably and accepts environment overrides.

### 5. Verify third-party assets

- Elephant Robotics robot, adaptive-gripper, and suction CAD retain local BSD-3-Clause attribution files.
- The vendored Three.js module retains its MIT license.

### 6. Review community health files

The repository includes:

- [`CONTRIBUTING.md`](CONTRIBUTING.md)
- [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md)
- [`SECURITY.md`](SECURITY.md)
- [`SUPPORT.md`](SUPPORT.md)
- [issue templates](.github/ISSUE_TEMPLATE)
- [pull-request template](.github/pull_request_template.md)
- [CODEOWNERS](.github/CODEOWNERS)

The root [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE) are included.

### 7. Run release checks

```bash
PYTHONPATH=. python3 -m unittest discover -s tests
git diff --check
git status --short
```

Also:

- install from a fresh clone;
- start without hardware;
- verify printable PDF links;
- verify all README links;
- confirm no workcell or key is exposed;
- perform separately documented staged physical validation.

---

## Contributing

Contributions are welcome under the project's Apache-2.0 terms.

### Good contributions

- reproducible bug fixes;
- regression tests;
- calibration diagnostics;
- safer failure behavior;
- documentation improvements;
- portable camera enumeration;
- new tool profiles with measured geometry;
- optional motion backends;
- accessibility and responsive UI improvements.

### Open an issue

Include:

- operating system and Python version;
- robot model and firmware;
- end effector;
- camera model and resolution;
- exact steps to reproduce;
- expected and actual behavior;
- complete error text;
- whether physical motion occurred;
- sanitized logs;
- screenshots or recorded frames when relevant;
- confirmation that secrets and device identifiers were removed.

### Submit a pull request

1. Create a focused branch.
2. Preserve existing user data and migrations.
3. Add or update tests.
4. Run the complete offline suite.
5. Run `git diff --check`.
6. Describe safety impact.
7. State what was and was not physically tested.
8. Do not include personal `data/workcell.json`.
9. Do not loosen validation merely to make a test pass.

### Development principles

- Reject unavailable coordinates instead of fabricating them.
- Keep classification separate from geometry.
- Keep robot-base, flange, TCP, and tool-contact frames explicit.
- Preserve meter/radian internals and convert only at firmware boundaries.
- Treat external state as stale until verified.
- Prefer deterministic planning over model-generated coordinates.
- Keep physical motion behind confirmation and feedback verification.
- Make automation propose, test, and report—not silently merge.

### Support

Use GitHub Issues for reproducible bugs and GitHub Discussions for setup questions once enabled. Do not send private support requests containing API keys, device identifiers, or unsafe physical-run instructions.

---

## Known limitations

- Camera localization assumes a planar table.
- Object tags must be on a planar top face.
- Object height is entered by the operator.
- Arbitrary side-mounted tags are not supported.
- Untagged automatic object creation is intentionally disabled.
- Physical obstacle avoidance is limited to modeled workspace geometry and planned exclusions.
- The host kinematic model must still be validated against each physical robot.
- Firmware coordinate behavior can differ across firmware versions.
- Suction seal detection is not available.
- Part mass is not measured.
- The adaptive gripper does not provide force feedback through this application.
- Physical partial execution and Move Here are intentionally unavailable.
- The HTTP server has no remote-user authentication and therefore refuses
  non-loopback network binding.
- macOS camera discovery has the most specialized support.
- No ROS 2 backend exists yet.

---

## ROS 2

The current platform does not require ROS 2. It is appropriate for a focused single-robot tabletop workcell.

A future ROS 2 / MoveIt 2 backend could add:

- standardized TF frames;
- planning-scene collision checking;
- trajectory planning and time parameterization;
- RViz debugging;
- rosbag recording;
- broader sensor and conveyor integration;
- multi-robot interoperability.

The recommended migration path is hybrid:

```text
Dashboard + camera + AI + programs
                |
        motion backend interface
          /                 \
 current pymycobot       future ROS 2
```

Do not rewrite the dashboard or camera registry merely to adopt ROS. Add ROS as an optional motion backend after validating the official myCobot URDF, tool transforms, limits, and controller behavior.

---

## License and third-party assets

### Project license

CobotWorkcell's original source code and documentation are licensed under the
[Apache License 2.0](LICENSE). See [`NOTICE`](NOTICE) for project attribution.

Files under `static/vendor/` and any other explicitly attributed third-party
components retain their original licenses. The root Apache-2.0 license does not
replace or override those terms.

### Elephant Robotics assets

The robot, adaptive-gripper, and suction CAD assets originate from Elephant
Robotics' BSD-3-Clause `mycobot_ros` repository. Source attribution and license
copies are stored alongside each asset group:

- [`static/vendor/mycobot_280_m5/README.md`](static/vendor/mycobot_280_m5/README.md)
- [`static/vendor/mycobot_280_m5/LICENSE.BSD-3-Clause`](static/vendor/mycobot_280_m5/LICENSE.BSD-3-Clause)
- [`static/vendor/adaptive_gripper/README.md`](static/vendor/adaptive_gripper/README.md)
- [`static/vendor/adaptive_gripper/LICENSE.BSD-3-Clause`](static/vendor/adaptive_gripper/LICENSE.BSD-3-Clause)
- [`static/vendor/suction_gripper/README.md`](static/vendor/suction_gripper/README.md)
- [`static/vendor/suction_gripper/LICENSE.BSD-3-Clause`](static/vendor/suction_gripper/LICENSE.BSD-3-Clause)

Three.js and its bundled modules retain the MIT license:

- [`static/vendor/three/LICENSE.MIT`](static/vendor/three/LICENSE.MIT)

Review every additional vendor asset before public redistribution.

### Trademarks

Elephant Robotics, myCobot, product names, and associated marks belong to their respective owners. This repository is an independent project unless explicitly stated otherwise.

---

## Acknowledgements

- [Elephant Robotics](https://www.elephantrobotics.com/) for the myCobot platform, `pymycobot`, robot descriptions, and accessory resources.
- [OpenCV](https://opencv.org/) for camera calibration, ChArUco, and AprilTag detection.
- [Three.js](https://threejs.org/) for browser-based 3D visualization.
- [OpenAI](https://openai.com/) for the optional Realtime assistant.
- The open-source robotics community for established practices around explicit frames, deterministic planning, simulation, testing, and review.

---

## Final operator checklist

Before simulation:

- [ ] Correct scene, tool, parts, bins, and points
- [ ] Current program saved
- [ ] Complete preview passes
- [ ] Full simulation inspected

Before physical execution:

- [ ] Correct robot and port
- [ ] Correct physical end effector
- [ ] Clear workspace
- [ ] Known payload
- [ ] Camera/tag pose current
- [ ] Tracked part visible
- [ ] Bin position verified
- [ ] Tool calibration current
- [ ] Reduced speed selected
- [ ] Stop available
- [ ] Explicit confirmation given only after inspection

If any item is uncertain, do not run the robot.
