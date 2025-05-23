function result = wb_accelerometer_get_values(tag)
% Usage: wb_accelerometer_get_values(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/accelerometer">here</a>

coder.extrinsic('calllib');
coder.extrinsic('setdatatype');
coder.extrinsic('get');
coder.extrinsic('libpointer');

%obj=libpointer("doublePtr",[0,0,0])
obj =calllib('libController', 'wb_accelerometer_get_values', tag);
disp(class(obj));

%setdatatype(obj,'doublePtr', 1, 3);
%disp(obj(1));
%disp(get(obj))
setdatatype(obj, 'doublePtr', 1, 3);  % 3 elemanlı bir double dizisi

values = get(obj,"Value");  % Verileri al

%disp(values);
result = values;
%result = get(obj, 'Value');  % Değerleri al