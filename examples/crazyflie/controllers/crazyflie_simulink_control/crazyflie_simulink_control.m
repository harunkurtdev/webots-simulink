% MATLAB controller for Webots
% File:          crazyflie_simulink_control.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:

desktop;
%keyboard;

TIME_STEP = 16;

imu = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(imu, TIME_STEP);

gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP);

range_front = wb_robot_get_device('range_front');
wb_distance_sensor_enable(range_front, TIME_STEP);

range_left = wb_robot_get_device('range_left');
wb_distance_sensor_enable(range_left, TIME_STEP);

range_back = wb_robot_get_device('range_back');
wb_distance_sensor_enable(range_back, TIME_STEP);

range_right = wb_robot_get_device('range_right');
wb_distance_sensor_enable(range_right, TIME_STEP);


m1_motor = wb_robot_get_device('m1_motor');
m2_motor = wb_robot_get_device('m2_motor');
m3_motor = wb_robot_get_device('m3_motor');
m4_motor = wb_robot_get_device('m4_motor');


%% https://www.mathworks.com/matlabcentral/fileexchange/65469-crazyflie-quadcopter-simulation-using-simmechanics

% Parametreler
g = 9.80665;
l1 = 0.0465; % m (cm'den dönüştürüldü)
l2 = 0.045;   % m
m_drone = 0.03097; % kg
Ix = 1.112951e-5; % kg*m²
Iy = 1.1143608e-5;
Iz = 2.162056e-5;
b_roll = 1e-5; % Tahmini açısal sönümleme
b_pitch = 1e-5;
b_yaw = 1e-5;
b_drone = 1e-9; % Translasyonel sönüm
km = 3.5077E-10; % Motor thrust coefficient

% State-Space Matrisleri
A = zeros(8,8);
A(1:3,4:6) = eye(3);
A(4,4) = -b_roll/Ix;
A(5,5) = -b_pitch/Iy;
A(6,6) = -b_yaw/Iz;
A(7,8) = 1;
A(8,8) = -b_drone/m_drone;

B = zeros(8,4);
B(4,2) = l1/Ix;   % Roll momenti
B(5,3) = l2/Iy;    % Pitch momenti
B(6,4) = 1/Iz;    % Yaw momenti
B(8,1) = 1/m_drone;% Thrust

C = eye(8);
D = zeros(8,4);


open_system('simulink_control');
load_system('simulink_control');
