function wb_motor_set_torque(tag, velocity)
% Usage: wb_motor_set_torque(tag, velocity)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motor">here</a>

coder.extrinsic('calllib');
calllib('libController', 'wb_motor_set_torque', tag, velocity);
