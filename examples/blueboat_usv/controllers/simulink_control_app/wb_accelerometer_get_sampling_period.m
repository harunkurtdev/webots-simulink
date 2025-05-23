function result = wb_accelerometer_get_sampling_period(tag)
% Usage: wb_accelerometer_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/accelerometer">here</a>

coder.extrinsic('calllib');
result = calllib('libController', 'wb_accelerometer_get_sampling_period', tag);
