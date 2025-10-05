%desktop;
%keyboard;

TIME_STEP = 16;
alpha_pitch=0.85;
alpha_roll=0.85;
alpha_yaw=0.85;

accelerometer_sensor=wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer_sensor,TIME_STEP);

gyro_sensor=wb_robot_get_device('gyro');
wb_gyro_enable(gyro_sensor,TIME_STEP);


gps_sensor=wb_robot_get_device('gps');
wb_gps_enable(gps_sensor,TIME_STEP);


inertial_unit=wb_robot_get_device('inertial_unit');
wb_inertial_unit_enable(inertial_unit,TIME_STEP)

rotational_motor = wb_robot_get_device('rotational_motor');
rotational_motor_propeller = wb_robot_get_device('rotational_motor_propeller');

position_sensor = wb_robot_get_device('position_sensor');

open_system('simulink_control');
load_system('simulink_control');
