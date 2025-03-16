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

% --- crazyflie parameters
g = 9.80665;
l1 = 4.65; %cm  1/2 of length of body
l2 = 4.5;
%Rotor/Propeller parameters
m = 0.00020; %propellor mass  kg
J = 1/12*m*(0.1^2+0.01^2); %moment of inertia from mass, kgm^2
b = 3.5077E-10; %motor viscous friction constant Nms
%drone parameters
m_drone = 0.03097;   % kg
Ix = 1.112951*10^-5;  % kg*m^2
Iy = 1.1143608*10^-5;  % kg*m^2
Iz = 2.162056*10^-5;  % kg*m^2
b_drone = 1*10^-9; % kg*m^2 drone's x,y,z translational drag coefficient


l = 0.0465; % m (Arm length)
k = 3.5077E-10; % Motor thrust coefficient

% State Variables: [x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi]
A = zeros(12,12);
A(1,7) = 1; A(2,8) = 1; A(3,9) = 1;
A(4,10) = 1; A(5,11) = 1; A(6,12) = 1;
A(7,5) = g; A(8,4) = -g;

B = zeros(12,4);
B(9,:) = k/m_drone; % Thrust input to acceleration in z
B(10,1) = l/Ix; B(10,3) = -l/Ix; % Torque in x
B(11,2) = l/Iy; B(11,4) = -l/Iy; % Torque in y
B(12,1) = k/Iz; B(12,2) = -k/Iz; B(12,3) = k/Iz; B(12,4) = -k/Iz; % Torque in z

C = eye(12); % All states are observable
D = zeros(12,4);


open_system('simulink_control');
load_system('simulink_control');
