function result = wb_gps_get_values(tag)
% Usage: wb_gps_get_values(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gps">here</a>

coder.extrinsic('calllib');
coder.extrinsic('setdatatype');
coder.extrinsic('get');
coder.extrinsic('libpointer');

obj =calllib('libController', 'wb_gps_get_values', tag);
disp(class(obj));

setdatatype(obj, 'doublePtr', 1, 3);  % 3 elemanlı bir double dizisi

values = get(obj,"Value");  % Verileri al

result = values;