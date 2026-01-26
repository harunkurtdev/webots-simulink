function result = wb_gyro_get_values(tag)
% Usage: wb_gyro_get_values(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gyro">here</a>

coder.extrinsic('calllib');
coder.extrinsic('setdatatype');
coder.extrinsic('get');
coder.extrinsic('libpointer');

%obj=libpointer("doublePtr",[0,0,0])
obj =calllib('libController', 'wb_gyro_get_values', tag);


setdatatype(obj, 'doublePtr', 1, 3);  % 3 elemanlı bir double dizisi

values = get(obj,"Value");  % Verileri al

result = values;
%result = get(obj, 'Value');  % Değerleri al