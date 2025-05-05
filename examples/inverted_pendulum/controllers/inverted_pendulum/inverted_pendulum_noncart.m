% MATLAB controller for Webots
% File:          inverted_pendulum_noncart.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:


% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
%keyboard;

TIME_STEP = 16;

horizontal_position_sensor = wb_robot_get_device('horizontal position sensor');
wb_position_sensor_enable(horizontal_position_sensor,TIME_STEP);

hip = wb_robot_get_device('hip');
wb_position_sensor_enable(hip,TIME_STEP);

horizontal_motor = wb_robot_get_device('horizontal_motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination


M=1;
m=0.05;
b=0.000001;
g=9.81;
l=0.3;
I=m*l^2;
i=I; % inertia


p = i*(M+m)+M*m*l^2; 


A = [0      1              0           0;
     0 -(i+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
     
B = [     0; 
     (i+m*l^2)/p;
          0;
        m*l/p];
        
C = [1 0 0 0;
     0 0 1 0];
     
D = [0;
     0];

%
open_system('simulink_control');
load_system('simulink_control');
