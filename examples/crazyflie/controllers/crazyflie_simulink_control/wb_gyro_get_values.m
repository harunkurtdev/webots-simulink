function result = wb_gyro_get_values(tag)
% Usage: wb_gyro_get_values(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gyro">here</a>


coder.extrinsic('calllib');
coder.extrinsic('setdatatype');
coder.extrinsic('get');


obj = calllib('libController', 'wb_gyro_get_values', tag);
setdatatype(obj,'doublePtr', 1, 3);
result = get(obj, 'Value');