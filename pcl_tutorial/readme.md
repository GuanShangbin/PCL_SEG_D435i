###realsense 体素滤波前后对比
roslaunch realsense2_camera rs_camera.launch
rosrun my_pcl_tutorial RectBox input:=/camera/depth_registered/points

