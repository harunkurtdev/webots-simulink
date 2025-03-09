function wb_motor_set_available_force(tag, force)

coder.extrinsic('calllib');
calllib('libController', 'wb_motor_set_available_force', tag, force);
