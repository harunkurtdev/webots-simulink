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

FRONT_WHEEL_RADIUS = 0.38;
REAR_WHEEL_RADIUS = 0.6;


accelerometer_sensor=wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer_sensor,TIME_STEP);

lidar_sensor=wb_robot_get_device('Sick LMS 291');
wb_lidar_enable(lidar_sensor,TIME_STEP);

compass_sensor=wb_robot_get_device('compass');
wb_compass_enable(compass_sensor,TIME_STEP);


gyro_sensor=wb_robot_get_device('gyro');
wb_gyro_enable(gyro_sensor,TIME_STEP);


gps_sensor=wb_robot_get_device('gps');
wb_gps_enable(gps_sensor,TIME_STEP);

right_steer_sensor=wb_robot_get_device('right_steer_sensor');
left_steer_sensor=wb_robot_get_device('left_steer_sensor');

wb_position_sensor_enable(right_steer_sensor,TIME_STEP);
wb_position_sensor_enable(left_steer_sensor,TIME_STEP);

left_front_wheel = wb_robot_get_device('left_front_wheel');
right_front_wheel = wb_robot_get_device('right_front_wheel');
left_rear_wheel = wb_robot_get_device('left_rear_wheel');
right_rear_wheel = wb_robot_get_device('right_rear_wheel');


left_steer = wb_robot_get_device('left_steer');
right_steer = wb_robot_get_device('right_steer');

left_flasher = wb_robot_get_device('left_flasher');
right_flasher = wb_robot_get_device('right_flasher');
tail_lights = wb_robot_get_device('tail_lights');
work_head_lights = wb_robot_get_device('work_head_lights');
road_head_lights = wb_robot_get_device('road_head_lights');

open_system('simulink_control');
load_system('simulink_control');
