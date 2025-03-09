function result = wb_inertial_unit_get_roll_pitch_yaw(tag)
% Usage: wb_accelerometer_get_values(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/accelerometer">here</a>

coder.extrinsic('calllib');
coder.extrinsic('setdatatype');
coder.extrinsic('get');

obj =calllib('libController', 'wb_inertial_unit_get_roll_pitch_yaw', tag);
disp(class(obj));

setdatatype(obj, 'doublePtr', 1, 3);  

values = get(obj,"Value");

disp(values);
result = values;