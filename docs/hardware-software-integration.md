# ASV 2026 — Hardware & Software Integration

## 1. Overview

The Autonomous Surface Vehicle (ASV) integrates hardware components and software modules to support autonomous navigation, environmental perception, vessel control, and real-time monitoring.

The system combines an autopilot platform, GPS positioning, camera systems, ROS 2 middleware, and a web-based monitoring interface.

## 2. Hardware Components

### 2.1 Pixhawk Autopilot
- Acts as the vessel's autopilot platform.
- Handles vehicle navigation and flight-controller communication.
- Interfaces with the navigation and control software through MAVROS.

### 2.2 GPS Module
- Provides geographical positioning information.
- Supports GPS-based waypoint navigation.
- Supplies position information for vessel tracking.

### 2.3 Camera Systems
- Surface camera for surface-level visual monitoring.
- Underwater camera for underwater visual monitoring.
- Provides visual data for the monitoring interface and computer vision system.

### 2.4 Propulsion System
- Electronic Speed Controllers (ESCs).
- Electric propulsion motors.
- Motor control through the vessel control system.

## 3. Software Components

| Component | Function |
|---|---|
| ROS 2 | Robotics middleware and node communication |
| MAVROS | Communication between ROS 2 and Pixhawk |
| Python | Autonomous system processing and integration |
| OpenCV | Image processing |
| YOLO | Computer vision and object detection |
| Fuzzy Logic | Autonomous control processing |
| Supabase | Cloud database and telemetry storage |
| HTML, CSS, JavaScript | Monitoring website interface |
| Chart.js | Data visualization |

## 4. System Integration Architecture

The system operates through several interconnected layers:

### A. Hardware Layer
- Pixhawk autopilot.
- GPS receiver.
- Surface and underwater cameras.
- ESC and propulsion motors.

### B. Robotics Processing Layer
- ROS 2 node communication.
- MAVROS integration.
- GPS waypoint navigation.
- Computer vision processing.
- Fuzzy Logic-based control.

### C. Data Communication Layer
- Python-based ROS 2 upload nodes.
- Telemetry data transmission.
- Processed data transmission.
- Supabase database integration.

### D. Monitoring Layer
- Web-based ASV dashboard.
- Live camera monitoring interface.
- GPS position visualization.
- SOG and COG visualization.
- Vessel trajectory monitoring.
- Mission A and Mission B interfaces.

## 5. Data Communication Flow

The telemetry and processing data flow follows this architecture:

1. Hardware components provide positioning and visual information.
2. ROS 2 nodes process sensor information and autonomous system data.
3. ROS 2 generates telemetry and processed results.
4. Python upload nodes transmit the relevant data to Supabase.
5. The monitoring website retrieves available data from Supabase.
6. The dashboard presents vessel information and mission monitoring data.

## 6. Monitoring Website Integration

The ASV monitoring website provides a graphical interface for observing vessel information.

Its main features include:

- Surface camera monitoring.
- Underwater camera monitoring.
- GPS position and geotag information.
- Speed Over Ground (SOG).
- Course Over Ground (COG).
- Vessel trajectory visualization.
- Mission A and Mission B interfaces.

The frontend uses Supabase as its data backend and Chart.js for visualization.

## 7. Integration Summary

The ASV project demonstrates the integration of embedded hardware, robotics middleware, autonomous processing, computer vision, data communication, and web-based monitoring.

This architecture enables different subsystems to work together as part of an autonomous surface vehicle platform.

---

*ASV 2026 — Team Varuna, Universitas Brawijaya.*
