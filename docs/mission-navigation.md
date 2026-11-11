
# ASV 2026 — Mission & Navigation Strategy

## 1. Overview

The Autonomous Surface Vehicle (ASV) supports two
navigation configurations: Mission A and Mission B.

Both missions use the same autonomous navigation
system, control architecture, and mission objectives.
The primary difference is the navigation route and
waypoint configuration.

## 2. Mission Configuration

### Mission A

Mission A uses a predefined navigation route
starting from the designated Start/Finish area.

The vessel follows the Mission A waypoint sequence
to complete the competition course.

### Mission B

Mission B uses a different predefined navigation
route with a different waypoint sequence.

The vessel follows the Mission B route while
utilizing the same navigation and control system
implemented in Mission A.

## 3. Mission Stages

The monitoring system tracks the following
competition mission stages:

1. Preparation
   - Monitoring system preparation.
   - Communication configuration.

2. Start
   - Beginning of autonomous mission execution.

3. Floating Ball Set 1–10
   - Navigation through the floating ball course.

4. Mission Surface Imaging
   - Surface imaging data generated and displayed
     on the monitoring interface.

5. Mission Underwater Imaging
   - Underwater imaging data generated and displayed
     on the monitoring interface.

6. Docking
   - Vessel docking operation.

7. Finish
   - Completion of the competition mission.

## 4. Navigation and Control

The ASV navigation system integrates:

- GPS-based waypoint navigation.
- Pixhawk autopilot integration.
- ROS 2 processing and communication.
- Fuzzy Logic-based control.
- Mission-specific route configuration.

Mission A and Mission B use the same control
architecture, with route configuration determining
the navigation sequence.

## 5. Monitoring and Telemetry

The monitoring interface provides information
about vessel position, movement, and mission progress.

### Position and Geotag Information

- Day and date.
- Time.
- GPS coordinates.
- Speed Over Ground (SOG).
- Course Over Ground (COG).

### Attitude Information

- Trajectory graph.
- Speed Over Ground (SOG).
- Course Over Ground (COG).

### Additional Monitoring Indicators

- Battery level.
- Visual video feed.

## 6. Mission Monitoring Interface

The web-based monitoring dashboard supports
both Mission A and Mission B configurations.

It provides:
- Mission progress and score monitoring.
- Vessel trajectory visualization.
- Navigation telemetry.
- Surface and underwater camera monitoring.
- Position and geotag information.

## 7. Mission Execution Flow

Preparation
    ↓
Start
    ↓
Waypoint Navigation
    ↓
Floating Ball Set 1–10
    ↓
Surface Imaging
    ↓
Underwater Imaging
    ↓
Docking
    ↓
Finish

The navigation route is selected according
to the assigned competition mission.

---

*Mission A and Mission B share the same
autonomous navigation and control architecture.
Only their navigation routes differ.*
