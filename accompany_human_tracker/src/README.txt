

rosparam set use_sim_time true

rosrun image_transport republish theora in:=/camera_kitchen/image_raw raw out:=/camera_kitchen/image_raw

rosrun image_transport republish theora in:=/camera_sofa/image_raw raw out:=/camera_sofa/image_raw

cd /home/bterwijn/projects/ros/accompany/accompany_uva/res/Heerlen
rosbag play -d 1 --clock -l 2013-06-14-13-59-27.bag --topics /camera_kitchen/camera_info /camera_kitchen/image_raw/theora /camera_sofa/camera_info /camera_sofa/image_raw/theora /map /tf

cd /home/bterwijn/projects/ros/accompany/UvA/Heerlen/cameras
roslaunch start.launch
