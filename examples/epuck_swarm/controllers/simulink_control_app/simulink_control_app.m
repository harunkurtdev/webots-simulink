% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   E-puck Swarm Robot Simulink Control
% Author:        Harun Kurt
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

TIME_STEP = 64;

% Swarm behavior parameters
AGGREGATION_WEIGHT = 1.0;
DISPERSION_WEIGHT = 0.5;
MAX_SPEED = 6.28;  % rad/s
OBSTACLE_THRESHOLD = 80;

% Initialize proximity sensors (8 IR sensors)
ps = zeros(1, 8);
for i = 0:7
    ps(i+1) = wb_robot_get_device(['ps' num2str(i)]);
    wb_distance_sensor_enable(ps(i+1), TIME_STEP);
end

% Initialize light sensors
ls = zeros(1, 8);
for i = 0:7
    ls(i+1) = wb_robot_get_device(['ls' num2str(i)]);
    wb_light_sensor_enable(ls(i+1), TIME_STEP);
end

% Initialize motors
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);

% Initialize communication devices
emitter = wb_robot_get_device('emitter');
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver, TIME_STEP);

% Initialize LEDs for visual feedback
leds = zeros(1, 8);
for i = 0:7
    leds(i+1) = wb_robot_get_device(['led' num2str(i)]);
end

% Get robot name for identification
robot_name = wb_robot_get_name();

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
