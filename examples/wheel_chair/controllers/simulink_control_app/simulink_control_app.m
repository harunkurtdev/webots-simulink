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
alpha_pitch=0.85

accelerometer_sensor=wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer_sensor,TIME_STEP);

gyro_sensor=wb_robot_get_device('gyro');
wb_gyro_enable(gyro_sensor,TIME_STEP);


left_IR = wb_robot_get_device('left_ir');
right_IR = wb_robot_get_device('right_ir');

% wb_distance_sensor_enable("left_ir", sampling_period)

wb_distance_sensor_enable(left_IR,TIME_STEP);
wb_distance_sensor_enable(right_IR,TIME_STEP);
  

inertial_unit=wb_robot_get_device('inertial_unit');
wb_inertial_unit_enable(inertial_unit,TIME_STEP)

left_position_sensor = wb_robot_get_device('left_position_sensor');
left_rotational_motor = wb_robot_get_device('left_rotational_motor');

right_position_sensor = wb_robot_get_device('right_position_sensor');
right_rotational_motor = wb_robot_get_device('right_rotational_motor');

open_system('simulink_control');
load_system('simulink_control');
