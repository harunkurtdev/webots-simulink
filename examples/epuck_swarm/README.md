# E-puck Swarm Robotics

This example demonstrates swarm robotics using multiple E-puck robots with MATLAB/Simulink control. The E-puck is a small differential wheeled mobile robot designed for education and research.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Swarm Robotics / Multi-Robot System |
| **Robot** | E-puck (multiple instances) |
| **Difficulty** | Intermediate |
| **Control Method** | Decentralized / Behavior-based |
| **Communication** | IR proximity / Range and bearing |

---

## Features

- **Multiple Robot Coordination**: 5-10 E-puck robots working together
- **Decentralized Control**: Each robot runs independently
- **Swarm Behaviors**: Aggregation, dispersion, flocking, foraging
- **Proximity Sensing**: 8 IR sensors for obstacle detection
- **Inter-Robot Communication**: IR-based local communication
- **LED Indicators**: Visual feedback for robot states

---

## Project Structure

```
epuck_swarm/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m        # Main MATLAB controller
│       ├── simulink_control.slx          # Simulink control model
│       ├── state_space_modeling.slx      # State-space model
│       ├── wb_motor_set_velocity.m       # Motor velocity control
│       ├── wb_distance_sensor_get_value.m # Proximity sensors
│       ├── wb_led_set.m                  # LED control
│       ├── wb_receiver_get_data.m        # Communication receive
│       ├── wb_emitter_send.m             # Communication send
│       └── wb_robot_step.m               # Simulation step
└── worlds/
    └── swarm_arena.wbt                   # Multi-robot world
```

---

## E-puck Specifications

### Physical Properties

| Property | Value |
|----------|-------|
| Diameter | 70 mm |
| Height | 50 mm |
| Weight | 150 g |
| Max Speed | 0.13 m/s |
| Wheel Radius | 20.5 mm |
| Axle Length | 52 mm |

### Sensors

| Sensor | Webots Name | Quantity | Purpose |
|--------|-------------|----------|---------|
| Proximity IR | `ps0` - `ps7` | 8 | Obstacle detection |
| Light Sensors | `ls0` - `ls7` | 8 | Light following |
| Accelerometer | `accelerometer` | 1 | Motion detection |
| Camera | `camera` | 1 | Visual input |
| ToF Sensor | `tof` | 1 | Distance measurement |

### Actuators

| Actuator | Webots Name | Control Mode |
|----------|-------------|--------------|
| Left Motor | `left wheel motor` | Velocity (rad/s) |
| Right Motor | `right wheel motor` | Velocity (rad/s) |
| LEDs | `led0` - `led7` | On/Off |

---

## Swarm Behaviors

### 1. Aggregation

Robots gather in a common area:

```matlab
function [v_left, v_right] = aggregation_behavior(proximity_values)
    % Move towards other robots
    attraction = sum(proximity_values(1:4)) - sum(proximity_values(5:8));

    base_speed = 2.0;
    v_left = base_speed + attraction * 0.5;
    v_right = base_speed - attraction * 0.5;
end
```

### 2. Dispersion

Robots spread out evenly:

```matlab
function [v_left, v_right] = dispersion_behavior(proximity_values)
    % Move away from other robots
    repulsion = max(proximity_values);

    if repulsion > threshold
        % Turn away from closest robot
        left_sum = sum(proximity_values(1:4));
        right_sum = sum(proximity_values(5:8));

        if left_sum > right_sum
            v_left = -1.0; v_right = 1.0;
        else
            v_left = 1.0; v_right = -1.0;
        end
    else
        % Random walk
        v_left = 2.0; v_right = 2.0;
    end
end
```

### 3. Flocking (Reynolds Rules)

```matlab
function [v_left, v_right] = flocking_behavior(neighbors, own_state)
    % Separation: avoid crowding neighbors
    separation = compute_separation(neighbors);

    % Alignment: steer towards average heading
    alignment = compute_alignment(neighbors);

    % Cohesion: steer towards center of mass
    cohesion = compute_cohesion(neighbors);

    % Combine behaviors
    heading = w1*separation + w2*alignment + w3*cohesion;
    [v_left, v_right] = heading_to_velocities(heading);
end
```

### 4. Foraging

```matlab
function [v_left, v_right] = foraging_behavior(light_values, carrying_food)
    if ~carrying_food
        % Search for food (follow light)
        [v_left, v_right] = follow_light(light_values);
    else
        % Return to nest (move away from light)
        [v_left, v_right] = avoid_light(light_values);
    end
end
```

---

## Controller Implementation

### Initialization

```matlab
% simulink_control_app.m
TIME_STEP = 64;  % Typical for e-puck

% Initialize proximity sensors
ps = zeros(1, 8);
for i = 0:7
    ps(i+1) = wb_robot_get_device(['ps' num2str(i)]);
    wb_distance_sensor_enable(ps(i+1), TIME_STEP);
end

% Initialize motors
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);

% Initialize communication
emitter = wb_robot_get_device('emitter');
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver, TIME_STEP);

% Initialize LEDs
leds = zeros(1, 8);
for i = 0:7
    leds(i+1) = wb_robot_get_device(['led' num2str(i)]);
end
```

### Main Control Loop

```matlab
while wb_robot_step(TIME_STEP) ~= -1
    % Read proximity sensors
    prox_values = zeros(1, 8);
    for i = 1:8
        prox_values(i) = wb_distance_sensor_get_value(ps(i));
    end

    % Read messages from other robots
    neighbors = [];
    while wb_receiver_get_queue_length(receiver) > 0
        data = wb_receiver_get_data(receiver);
        neighbors = [neighbors; data];
        wb_receiver_next_packet(receiver);
    end

    % Compute swarm behavior
    [v_left, v_right] = swarm_behavior(prox_values, neighbors);

    % Apply motor commands
    wb_motor_set_velocity(left_motor, v_left);
    wb_motor_set_velocity(right_motor, v_right);

    % Broadcast own state
    own_state = [robot_id, x, y, theta];
    wb_emitter_send(emitter, own_state);
end
```

---

## World Configuration

The world file should contain multiple E-puck robots:

```
# swarm_arena.wbt (excerpt)
E-puck {
  translation 0 0 0
  name "e-puck_0"
  controller "simulink_control_app"
}
E-puck {
  translation 0.2 0 0
  name "e-puck_1"
  controller "simulink_control_app"
}
# ... more robots
```

---

## Quick Start

1. **Open Webots** and load `examples/epuck_swarm/worlds/swarm_arena.wbt`

2. **Configure MATLAB** path to include Webots libraries

3. **Select swarm behavior** in Simulink model

4. **Run simulation** and observe collective behavior

5. **Experiment** with different:
   - Number of robots
   - Behavior parameters
   - Arena configurations

---

## Performance Metrics

| Metric | Description |
|--------|-------------|
| **Convergence Time** | Time for swarm to reach goal state |
| **Dispersion Index** | Measure of spatial distribution |
| **Connectivity** | Graph connectivity of robot network |
| **Collision Rate** | Number of inter-robot collisions |
| **Energy Efficiency** | Total distance traveled vs. goal achieved |

---

## Applications

- **Search and Rescue**: Distributed area coverage
- **Environmental Monitoring**: Multi-point sensing
- **Warehouse Automation**: Coordinated transport
- **Education**: Multi-agent systems concepts

---

## References

- [E-puck Official Website](http://www.e-puck.org/)
- [Webots E-puck Documentation](https://cyberbotics.com/doc/guide/epuck)
- Reynolds, C. W. (1987). "Flocks, herds and schools: A distributed behavioral model"
- Brambilla, M. et al. (2013). "Swarm robotics: a review from the swarm engineering perspective"
