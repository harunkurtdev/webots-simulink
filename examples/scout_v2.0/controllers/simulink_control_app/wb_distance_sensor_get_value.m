function result = wb_distance_sensor_get_value(tag)

coder.extrinsic('calllib');
coder.extrinsic('setdatatype');
coder.extrinsic('get');
coder.extrinsic('libpointer');


result = calllib('libController', 'wb_distance_sensor_get_value', tag);