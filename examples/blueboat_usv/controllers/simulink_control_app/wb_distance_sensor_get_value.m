function result = wb_distance_sensor_get_value(tag)

coder.extrinsic('calllib');
result = calllib('libController', 'wb_distance_sensor_get_value', tag);