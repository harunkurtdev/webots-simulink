# Adding New Examples Guide

This guide explains how to add new robot examples to the Webots-Simulink Bridge project.

---

## Potential New Examples

Based on Webots' available robot models and common simulation use cases, here are recommended examples to add:

### Autonomous Vehicles

#### BMW X5 / Tesla Model 3
- **Type**: Passenger car with Ackermann steering
- **Sensors**: Camera, LiDAR, radar, GPS, IMU
- **Use Cases**: ADAS development, lane keeping, ACC, autonomous parking
- **Webots Model**: `BmwX5`, `TeslaModel3`

```
bmw_x5/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m
│       ├── simulink_control.slx
│       ├── wb_camera_get_image.m
│       ├── wb_lidar_get_range_image.m
│       ├── wb_radar_get_targets.m
│       └── ...
└── worlds/
    └── highway.wbt
```

#### Citroen C-Zero / Smart ForTwo
- **Type**: Compact electric vehicle
- **Use Cases**: Urban autonomous driving, parking algorithms

---

### Industrial Robots

#### Universal Robots (UR5e/UR10e)
- **Type**: 6-DOF collaborative robot arm
- **Sensors**: Joint encoders, force/torque sensor
- **Use Cases**: Pick-and-place, trajectory planning, inverse kinematics
- **Webots Model**: `UR5e`, `UR10e`

```
ur5e_arm/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m
│       ├── simulink_control.slx
│       ├── wb_motor_set_position.m
│       ├── wb_position_sensor_get_value.m
│       └── ...
└── worlds/
    └── ur5e_workspace.wbt
```

#### KUKA iiwa / ABB IRB
- **Type**: Industrial robot arm
- **Use Cases**: Manufacturing simulation, welding, assembly

---

### Humanoid Robots

#### NAO Robot
- **Type**: Bipedal humanoid (58cm)
- **Sensors**: Cameras, IMU, touch sensors, microphones
- **Use Cases**: Gait control, balance, gesture recognition
- **Webots Model**: `Nao`

```
nao_humanoid/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m
│       ├── simulink_control.slx
│       ├── wb_motor_set_position.m   # Joint control
│       ├── wb_camera_get_image.m
│       ├── wb_accelerometer_get_values.m
│       └── ...
└── worlds/
    └── nao_walking.wbt
```

#### Boston Dynamics Spot
- **Type**: Quadruped robot
- **Sensors**: Depth cameras, IMU, joint encoders
- **Use Cases**: Legged locomotion, terrain navigation

---

### Underwater Vehicles

#### BlueROV2
- **Type**: Remotely operated underwater vehicle
- **Sensors**: Depth sensor, IMU, cameras, sonar
- **Use Cases**: Underwater inspection, marine research

```
bluerov2/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m
│       ├── simulink_control.slx
│       ├── wb_motor_set_velocity.m   # Thruster control
│       ├── wb_distance_sensor_get_value.m  # Depth
│       └── ...
└── worlds/
    └── underwater.wbt
```

---

### Agricultural Robots

#### Autonomous Harvester
- **Type**: Agricultural vehicle with implement
- **Sensors**: GPS, LiDAR, cameras
- **Use Cases**: Precision agriculture, autonomous harvesting

#### Drone Sprayer
- **Type**: Agricultural multirotor
- **Use Cases**: Crop spraying, field mapping

---

### Swarm Robotics

#### E-puck Swarm
- **Type**: Multiple small mobile robots
- **Sensors**: Proximity sensors, cameras, IR communication
- **Use Cases**: Swarm algorithms, collective behavior

```
epuck_swarm/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m
│       ├── simulink_control.slx
│       └── ...
└── worlds/
    └── swarm_arena.wbt  # Multiple e-puck robots
```

---

## Example Structure Template

Every example should follow this structure:

```
example_name/
├── README.md                        # Quick start guide
├── controllers/
│   ├── keyboard_controller/         # Optional manual control
│   │   └── keyboard_controller.py
│   └── simulink_control_app/        # Main Simulink integration
│       ├── simulink_control_app.m   # Initialization script
│       ├── simulink_control.slx     # Main Simulink model
│       ├── state_space_modeling.slx # Optional state-space model
│       └── wb_*.m                   # MATLAB wrapper functions
├── protos/                          # Optional custom PROTO files
│   └── CustomRobot.proto
└── worlds/
    └── world.wbt                    # Webots world file
```

---

## Step-by-Step: Adding a New Example

### Step 1: Create Directory Structure

```bash
mkdir -p examples/new_robot/controllers/simulink_control_app
mkdir -p examples/new_robot/worlds
```

### Step 2: Create Webots World File

Create `worlds/world.wbt` with:
- Robot definition with sensors and actuators
- Environment (floor, obstacles, etc.)
- Set controller to `simulink_control_app`

### Step 3: Create Initialization Script

Create `simulink_control_app.m`:

```matlab
% MATLAB controller for Webots
% File: simulink_control_app.m
% Description: [Robot Name] Simulink Control Initialization
% Author: [Your Name]
% Date: [Date]

TIME_STEP = 16;

% ===== SENSORS =====
% Initialize each sensor your robot has

% IMU / Inertial Unit
imu = wb_robot_get_device('inertial_unit');
wb_inertial_unit_enable(imu, TIME_STEP);

% GPS
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% Gyroscope
gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

% LiDAR (if applicable)
lidar = wb_robot_get_device('lidar');
wb_lidar_enable(lidar, TIME_STEP);

% Camera (if applicable)
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP);

% ===== ACTUATORS =====
% Initialize motors/actuators

motor1 = wb_robot_get_device('motor1');
motor2 = wb_robot_get_device('motor2');
% ... add more as needed

% ===== LOAD SIMULINK MODEL =====
open_system('simulink_control');
load_system('simulink_control');
```

### Step 4: Create MATLAB Wrapper Functions

Copy and adapt wrapper functions from existing examples:

**Essential functions:**
- `wb_robot_step.m` - Simulation step
- `wb_motor_set_velocity.m` - Motor velocity control
- `wb_motor_set_position.m` - Motor position control

**Sensor functions (as needed):**
- `wb_gps_get_values.m`
- `wb_gyro_get_values.m`
- `wb_accelerometer_get_values.m`
- `wb_inertial_unit_get_roll_pitch_yaw.m`
- `wb_lidar_get_range_image.m`
- `wb_camera_get_image.m`
- `wb_distance_sensor_get_value.m`

### Step 5: Create Simulink Model

1. Create `simulink_control.slx`
2. Add MATLAB Function blocks for sensor reading
3. Add control algorithm blocks
4. Add MATLAB Function blocks for actuator commands
5. Configure sample time to match TIME_STEP

### Step 6: Create Documentation

**README.md** (in example folder):
```markdown
# [Robot Name]

![Robot Image](./worlds/.world.jpg)

Brief description of the robot.

## Features
- Feature 1
- Feature 2

## Quick Start
1. Open `worlds/world.wbt` in Webots
2. Set controller to `simulink_control_app`
3. Open `simulink_control.slx` in MATLAB
4. Run simulation

## File Structure
...

See [full documentation](../../docs/examples/robot-name.md) for details.
```

**Full documentation** (`docs/examples/robot-name.md`):
- System overview
- Specifications table
- Component tables (sensors, actuators)
- Simulink integration details
- Control system design
- Usage examples
- Applications
- References

### Step 7: Update Navigation

Add to `mkdocs.yml`:
```yaml
- Examples:
    - Category:
        - Robot Name: examples/robot-name.md
```

### Step 8: Test the Example

1. Open Webots world
2. Run MATLAB initialization
3. Start Simulink model
4. Verify sensor data flow
5. Test actuator commands
6. Document any issues

---

## MATLAB Wrapper Function Template

```matlab
function result = wb_sensor_get_value(tag)
% WB_SENSOR_GET_VALUE Get value from Webots sensor
%   result = wb_sensor_get_value(tag) returns the sensor value
%
%   Parameters:
%       tag - Device tag from wb_robot_get_device
%
%   Returns:
%       result - Sensor value (type depends on sensor)

    % Mark as extrinsic for Simulink code generation
    coder.extrinsic('calllib');

    % Initialize output
    result = 0;  % or zeros(1,3) for vector outputs

    % Call Webots API
    result = calllib('libController', 'wb_sensor_get_value', tag);
end
```

---

## Best Practices

1. **Consistent Naming**: Use snake_case for folders, kebab-case for docs
2. **Device Names**: Match exactly what's in the Webots world file
3. **TIME_STEP**: Use 16ms (62.5 Hz) for most applications
4. **Error Handling**: Add try-catch in critical functions
5. **Documentation**: Include screenshots and GIFs
6. **Testing**: Verify on multiple platforms (Windows, Linux, macOS)

---

## Checklist for New Examples

- [ ] Directory structure created
- [ ] Webots world file works standalone
- [ ] Initialization script runs without errors
- [ ] All wrapper functions tested
- [ ] Simulink model compiles and runs
- [ ] README.md created in example folder
- [ ] Full documentation in docs/examples/
- [ ] Added to mkdocs.yml navigation
- [ ] Screenshots/GIFs captured
- [ ] Tested on clean MATLAB installation
