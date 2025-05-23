% MATLAB controller for Webots
% File:          simulink_control.m
% Date: 
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
%keyboard;

TIME_STEP = 16;


accelerometer_sensor=wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer_sensor,TIME_STEP);

lidar_sensor=wb_robot_get_device('LDS-01');
wb_lidar_enable(lidar_sensor,TIME_STEP);

compass_sensor=wb_robot_get_device('compass');
wb_compass_enable(compass_sensor,TIME_STEP);


gyro_sensor=wb_robot_get_device('gyro');
wb_gyro_enable(gyro_sensor,TIME_STEP);

right_wheel_sensor=wb_robot_get_device('right_wheel_sensor');
left_wheel_sensor=wb_robot_get_device('left_wheel_sensor');

wb_position_sensor_enable(left_wheel_sensor,TIME_STEP);
wb_position_sensor_enable(right_wheel_sensor,TIME_STEP);

left_wheel_motor = wb_robot_get_device('left wheel motor');
right_wheel_motor = wb_robot_get_device('right wheel motor');


open_system('simulink_control');
load_system('simulink_control');
