function result = wb_robot_step(duration)

coder.extrinsic('calllib');
result = calllib('libController', 'wb_robot_step', duration);
