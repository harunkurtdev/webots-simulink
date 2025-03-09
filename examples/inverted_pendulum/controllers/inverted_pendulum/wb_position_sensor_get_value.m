function result = wb_position_sensor_get_value(tag)
% Usage: wb_position_sensor_get_value(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gyro">here</a>

coder.extrinsic('calllib');
obj =calllib('libController', 'wb_position_sensor_get_value', tag);

result = obj;