# AeroTHON

AeroTHON is a ROS 2 + PX4 autonomous drone mission stack for a corridor-navigation and payload-delivery scenario. The repository contains the flight controller, perception modules, mission logic, custom messages, and launch files needed to fly a drone through a mission workflow from takeoff to payload delivery and landing.

This repository is organized as a set of ROS packages under `src/`, with Python-based nodes for detection, flight control, and behavior orchestration. It is designed to run on PX4 SITL or real hardware, with ROS 2 as the middleware layer.

## High-level purpose

The project targets a mission pattern similar to:

- take off and reach a safe altitude,
- detect and center on a start QR or banner,
- navigate through a corridor,
- search for a payload target,
- align with the target or delivery zone,
- deliver the payload,
- return and land.

The codebase is structured around a central PX4 controller and a mission tree that executes actions step by step.

## Repository structure

```text
aerothon/
├── .gitignore
├── px4_msgs_v1.15_backup/            # Backup copy of PX4 message definitions
├── src/
│   ├── banner_align/                 # Aligns drone with the green corridor banner
│   ├── banner_detector/              # Detects green AeroTHON banner using HSV + OCR
│   ├── corridor_navigator/           # Corridor flight and geofence logic
│   ├── flight_controller/            # Main PX4 OFFBOARD flight controller
│   ├── launch/                       # Launch files for mission startup and SITL setup
│   ├── mission_interfaces/           # Custom ROS messages, services, and action definitions
│   ├── mission_management/           # Mission tree / behavior orchestration
│   ├── obstacle_detector/            # Obstacle detection around the corridor
│   ├── payload/                      # Payload release task
│   ├── px4_msgs/                     # PX4 ROS message definitions
│   ├── qr_detector/                  # QR detection and reference capture
│   ├── red_zone_detector/            # Restricted zone / red-region detection
│   ├── siyi_a8/                      # Siyi A8 camera / photo task integration
│   ├── takeoff/                      # Legacy/simple takeoff helper package
│   ├── target_centering/             # Centers the drone on target in camera frame
│   ├── target_search/                # Searches the region for a target matching the scanned QR
│   └── ...
└── README.md
```

## Main packages and what they do

### 1) `src/flight_controller`

This is the core of the drone control stack.

Files:

- `flight_controller/flight_controller_node.py`
  - Main controller node for PX4.
  - Creates action servers for `Takeoff`, `GoTo`, and `Land`.
  - Publishes OFFBOARD mode and trajectory setpoints to PX4.
  - Maintains vehicle state (`status`, `local_pos`, `amsl`, `landed`).
  - Handles command acknowledgements, mode switching, GPS/position checks, and action cancellation.
  - Ensures the drone only flies under OFFBOARD control when conditions are safe.

- `flight_controller/px4_utils.py`
  - Contains utility constants, topic wrappers, and PX4-related helpers.
  - Used to normalize PX4 topic names and command results.

Why it matters:

This is the only node that directly commands the PX4 autopilot in OFFBOARD mode. Every mission action eventually flows through this controller.

### 2) `src/mission_interfaces`

This package defines the custom ROS interfaces used throughout the project.

Files/directories:

- `action/*.action`
  - `Takeoff.action`
  - `GoTo.action`
  - `Land.action`
  - `NavigateCorridor.action`
  - `AlignWithCorridor.action`
  - `SearchTarget.action`
  - `CenterOnTarget.action`
  - `CaptureReference.action`
  - `DeliverPayload.action`
  - These define mission actions called by behavior tree nodes and controllers.

- `msg/*.msg`
  - `BannerDetection.msg`
  - `QrDetection.msg`
  - `QrCode.msg`
  - `Obstacles.msg`
  - `RestrictedZones.msg`
  - `RedPatch.msg`
  - These define messages for perception outputs like banner detection, obstacle reports, QR results, and no-fly regions.

- `srv/*.srv`
  - `SetPhotoTask.srv`
  - `SetZoom.srv`
  - Used for camera tasks and photo-trigger control.

Why it matters:

This package is the contract between all mission modules. If a node needs to ask the drone to take off, go somewhere, detect a target, or deliver payload, it does so through these ROS interfaces.

### 3) `src/mission_management`

This package coordinates the mission as a behavior tree.

Files:

- `mission_management/mission_tree.py`
  - Implements the overall mission as a `py_trees` behavior tree.
  - Waits until PX4 is ready and local/global position data is valid.
  - Executes the mission sequence: takeoff, QR scan, corridor alignment, corridor fly-through, target search, payload delivery, return route, landing.
  - Uses `WithPhotos` wrappers to turn the camera photo task on only while relevant perception steps run.
  - Marks some steps as placeholders where the behavior is not fully implemented yet.

- `mission_management_node.py`
  - Another mission-management node for older or simpler mission execution.

Why it matters:

This is the mission planner. It decides the order of operations and which actions to call.

### 4) `src/launch`

This directory contains launch configuration files.

Files:

- `mission.launch.py`
  - Old stack launch file.
  - Runs simpler mission nodes like `takeoff_node`, `move_forward`, and `mission_management_node`.
  - Used for the legacy mission flow.

- `autonomy.launch.py`
  - Newer launch file.
  - Starts: flight controller, target centering, QR detector, banner detector, obstacle detector, red zone detector, corridor navigator, target search, payload, and mission tree.
  - Also accepts parameters like `reference_qr_path` and `camera_hfov`.

- `drone.launch.py`
  - Likely starts the drone runtime or vehicle-related initialization.

- `sitl.launch.py`
  - Starts PX4 SITL and supporting launch pieces for simulation.

Why it matters:

The launch files are the entry point for starting everything in one command.

### 5) `src/banner_detector`

This module detects the green Aerothon corridor banner using image processing and OCR.

Files:

- `banner_detector/banner_detector.py`
  - Contains the main banner detection logic.
  - Uses HSV color thresholding to find green regions.
  - Filters contours by area and aspect ratio.
  - Tracks the most likely banner across frames.
  - Runs EasyOCR on promising regions to verify the banner text such as `AEROTHON 2026`.

- `banner_detector/banner_detector_node.py`
  - ROS 2 wrapper around the detector.
  - Subscribes to camera images.
  - Converts ROS images to NumPy/OpenCV arrays.
  - Publishes `/perception/banner` and debug image topics.

Why it matters:

This is used to detect alignment markers and corridor indicators while flying.

### 6) `src/qr_detector`

This package handles QR detection and reference image capture.

Files:

- `qr_detector/qr_detector_node.py`
  - Receives camera frames and detects QR codes.
  - Uses QR data to determine the drone’s target or reference markers.

- `qr_detector/move_forward.py`
  - A simple motion helper to move forward by a set distance and speed.
  - Used in the old mission stack.

- `config/qr_detector.yaml`
  - Configuration parameters for QR detection behavior.

Why it matters:

This module is responsible for identifying mission-relevant QR targets and capturing the starting reference image.

### 7) `src/target_centering`

This package tries to center the drone on the target in the camera view.

Files:

- `target_centering/target_centering_node.py`
  - Estimates the object’s location in the image.
  - Adjusts the drone motion to keep the target centered.
  - Uses the camera horizontal FOV to convert pixel error into control signals.

Why it matters:

A target-centered flight is necessary before payload delivery or target alignment.

### 8) `src/target_search`

This module searches for the target QR or target region in the assigned mission area.

Files:

- `target_search/target_search_node.py`
  - Searches for a target object or QR code.
  - Publishes or triggers actions when the target is found.

Why it matters:

This is used after corridor navigation to find the exact delivery target.

### 9) `src/corridor_navigator`

This provides the logic required to fly along a corridor while staying within a safe path.

Files:

- `corridor_navigator/corridor_logic.py`
  - Pure-Python geometry and control math.
  - Parses geofence/delivery-zone files.
  - Converts lat/lon to local coordinates.
  - Performs line projection, cross-track error minimization, and body-frame velocity calculation.
  - Uses gain-based commands to hold the line and adjust heading.

- `corridor_navigator/corridor_navigator_node.py`
  - ROS node that uses the logic to fly the drone along the corridor path.

- `config/corridor_navigator.params.yaml`
  - Parameter settings for corridor navigation.

- `config/delivery_zone.example.txt`
  - Example polygon file describing a delivery zone.

Why it matters:

This is the core guidance algorithm for corridor-following flight.

### 10) `src/banner_align`

This package aligns the drone against the banner so it can fly down the correct corridor.

Files:

- `banner_align/banner_align_node.py`
  - Receives banner detection results and steers the drone toward the correct alignment.
  - Keeps the banner centered and maintains a good approach line.

Why it matters:

It is the waypoint/visual servoing layer that links banner detection to stable drone motion.

### 11) `src/obstacle_detector`

This package is intended to detect obstacles in the corridor.

Files:

- `obstacle_detector/obstacle_detector_node.py`
  - Publishes obstacle information on `/perception/obstacles`.
  - Current implementation is a placeholder; detection logic is not completed yet.

Why it matters:

This is a planned safety layer for collision avoidance and corridor awareness.

### 12) `src/red_zone_detector`

This package detects restricted or red-marked zones.

Files:

- `red_zone_detector/red_zone_detector_node.py`
  - Intended to detect red patches or restricted area boundaries.
  - Publishes detection information to downstream mission logic.

Why it matters:

This supports safe flight restrictions and mission region validation.

### 13) `src/payload`

This package implements the payload delivery mechanism.

Files:

- `payload/payload_node.py`
  - Action server for `/deliver_payload`.
  - Currently logs an error and aborts because the hardware behavior is not implemented yet.

Why it matters:

This is the mechanism node for the final mission step where the payload is released.

### 14) `src/siyi_a8`

This package connects the drone to the Siyi A8 camera system and photo/task service.

Files:

- `siyi_a8/gimbal_node.py`
  - Controls camera gimbal orientation.

- `siyi_a8/photo_node.py`
  - Handles still image capture.

- `siyi_a8/rtsp_camera_node.py`
  - Streams or interfaces with an RTSP camera.

- `siyi_a8/siyi_protocol.py`
  - Implements the protocol for communication with Siyi hardware.

Why it matters:

This allows the drone to capture reference images and mission-related photos during task execution.

### 15) `src/takeoff`

This is a smaller legacy package for takeoff tasks.

Files:

- `takeoff/takeoff_node.py`
  - Starts a simple takeoff action or behavior.
  - It is part of the older mission flow.

Why it matters:

This package is kept for compatibility with the older mission stack.

### 16) `src/px4_msgs`

This directory contains PX4 ROS message definitions generated for use with ROS 2.

Files:

- `msg/*.msg`
- `srv/VehicleCommand.srv`
- `CMakeLists.txt`, `package.xml`

Why it matters:

The flight controller relies on these messages to communicate with PX4, including vehicle status, commands, trajectories, and telemetry.

### 17) `px4_msgs_v1.15_backup`

This is a backup copy of PX4 message definitions and services from an earlier version.

Why it matters:

Useful for rollbacks or compatibility checks when working with older PX4 message schemas.

## How the system works together

The project uses a ROS 2 architecture with a mission flow like this:

1. `mission_management` decides the sequence of actions.
2. `flight_controller` executes physical movement actions like takeoff, goto, and land through PX4.
3. Perception modules (`banner_detector`, `qr_detector`, `target_centering`, `target_search`) analyze camera data.
4. Guidance packages (`banner_align`, `corridor_navigator`) turn sensor data into control commands.
5. `mission_interfaces` defines the message/action format used between all nodes.
6. Launch files bring these packages together at runtime.

## Typical launch commands

Start SITL first, then run a mission:

```bash
ros2 launch src/launch/sitl.launch.py
ros2 launch src/launch/mission.launch.py
```

Or use the new autonomy stack:

```bash
ros2 launch src/launch/autonomy.launch.py
```

Example with a preloaded QR reference image:

```bash
ros2 launch src/launch/autonomy.launch.py reference_qr_path:=/abs/path/to/ref.png
```

## Notes and status

This repo is a real-world robotics project in active development. Some modules are fully implemented, while others are either partial, test scaffolding, or placeholders.

Examples:

- `flight_controller` is the most complete, central control layer.
- `banner_detector` includes substantial detection logic.
- `obstacle_detector`, `payload`, and some mission return steps are clearly not fully implemented yet.
- `mission_management/mission_tree.py` intentionally includes placeholder return/unfinished steps.

## Summary

AeroTHON is a ROS 2-based autonomous drone mission stack designed to:

- detect navigation markers and QR targets,
- fly a corridor safely,
- execute a target search and payload delivery task,
- coordinate actions through a central mission tree,
- and interface directly with PX4 for flight control.

This project shows a modular robotics architecture: perception + navigation + action execution + mission orchestration. The repository is organized around independent ROS packages, each focused on a different part of the mission pipeline.

## License and project context

The project is a custom autonomous drone stack built around ROS 2 and PX4. It is intended to run in simulation and on real hardware under the same mission architecture.

If you want, I can also generate:

- a more polished GitHub-style README with badges and screenshots,
- a shorter version for quick project overview,
- or a README tailored specifically for public GitHub presentation.
