function wb_lidar_enable_point_cloud(tag)
% Usage: wb_lidar_enable_point_cloud(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>
coder.extrinsic('calllib');
coder.extrinsic('setdatatype');
coder.extrinsic('get');
coder.extrinsic('libpointer');

calllib('libController', 'wb_lidar_enable_point_cloud', tag);
