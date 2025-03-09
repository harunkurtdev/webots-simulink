function wb_motor_set_position(tag, position)

coder.extrinsic('calllib');
calllib('libController', 'wb_motor_set_position', tag, position);
