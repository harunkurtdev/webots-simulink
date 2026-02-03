% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   UR5e Robot Arm Simulink Control
% Author:        Harun Kurt
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

TIME_STEP = 16;

% DH Parameters for UR5e
a = [0, -0.425, -0.3922, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

% Joint names
joint_names = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', ...
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

% Initialize motors and position sensors
motors = cell(1, 6);
sensors = cell(1, 6);

for i = 1:6
    motors{i} = wb_robot_get_device(joint_names{i});
    sensors{i} = wb_robot_get_device([joint_names{i} '_sensor']);
    wb_position_sensor_enable(sensors{i}, TIME_STEP);
end

% Home position (joint angles in radians)
home_position = [0, -pi/2, pi/2, -pi/2, -pi/2, 0];

% Move to home position
for i = 1:6
    wb_motor_set_position(motors{i}, home_position(i));
end

% Control parameters
Kp = 5.0;   % Position gain
Kd = 1.0;   % Velocity gain

% Target positions for demo
target_positions = [
    0, -pi/2, pi/2, -pi/2, -pi/2, 0;        % Home
    pi/4, -pi/3, pi/2, -pi/2, -pi/2, 0;     % Position 1
    -pi/4, -pi/3, pi/3, -pi/2, -pi/2, 0;    % Position 2
    0, -pi/2, pi/2, -pi/2, -pi/2, 0;        % Back to home
];

% Current target index
target_index = 1;

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
