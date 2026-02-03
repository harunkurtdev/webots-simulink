# Tesla Model 3 Autonomous Vehicle

This example demonstrates autonomous vehicle control using the Tesla Model 3 with MATLAB/Simulink. The simulation includes ADAS features, lane keeping, adaptive cruise control, and autonomous parking.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Autonomous Electric Vehicle |
| **Robot** | Tesla Model 3 |
| **Difficulty** | Intermediate to Advanced |
| **Control Method** | Ackermann Steering / MPC |
| **Sensors** | Camera, LiDAR, Radar, GPS, IMU |

---

## Features

- **Autonomous Driving**: Level 2+ ADAS capabilities
- **Lane Keeping**: Camera-based lane detection
- **Adaptive Cruise Control**: Radar-based distance control
- **Automatic Emergency Braking**: Obstacle detection
- **Autonomous Parking**: Parallel and perpendicular
- **Traffic Sign Recognition**: Vision-based detection

---

## Project Structure

```
tesla_model3/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m
│       ├── simulink_control.slx
│       ├── lane_detection.m
│       ├── path_planning.m
│       ├── vehicle_control.m
│       ├── wb_camera_get_image.m
│       ├── wb_lidar_get_range_image.m
│       └── wb_*.m
└── worlds/
    └── highway.wbt
```

---

## Vehicle Specifications

| Property | Value |
|----------|-------|
| Length | 4.694 m |
| Width | 1.849 m |
| Height | 1.443 m |
| Wheelbase | 2.875 m |
| Max Speed | 261 km/h |

---

## Sensors

| Sensor | Quantity | Purpose |
|--------|----------|---------|
| Front Camera | 1 | Lane detection, signs |
| Side Cameras | 2 | Blind spot monitoring |
| Rear Camera | 1 | Reverse assist |
| Front Radar | 1 | ACC, AEB |
| LiDAR | 1 | 3D environment mapping |
| GPS | 1 | Localization |
| IMU | 1 | Vehicle dynamics |

---

## Control Architecture

### Lateral Control (Steering)
- Pure Pursuit controller
- Stanley controller
- MPC-based steering

### Longitudinal Control (Speed)
- PID cruise control
- ACC with radar feedback
- Regenerative braking

---

## Quick Start

1. Load `examples/tesla_model3/worlds/highway.wbt`
2. Configure MATLAB controller
3. Run simulation
4. Observe autonomous driving behavior

---

## ADAS Features

### Lane Keeping Assist (LKA)
```matlab
function steering = lane_keeping(camera_image)
    % Detect lane lines
    lanes = detect_lanes(camera_image);

    % Calculate lateral error
    lateral_error = lanes.center - image_center;

    % PID steering
    steering = Kp * lateral_error + Kd * lateral_error_dot;
end
```

### Adaptive Cruise Control (ACC)
```matlab
function throttle = adaptive_cruise(radar_data, set_speed)
    distance = radar_data.distance;
    relative_speed = radar_data.relative_speed;

    if distance < safe_distance
        throttle = -Kp * (safe_distance - distance);
    else
        throttle = Kp * (set_speed - current_speed);
    end
end
```

---

## References

- Tesla Autopilot Documentation
- [Webots Tesla Model 3](https://cyberbotics.com/doc/automobile/car)
- Paden, B. et al. (2016). "A Survey of Motion Planning and Control Techniques"
