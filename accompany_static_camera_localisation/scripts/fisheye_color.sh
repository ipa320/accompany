roscore &
rosrun gscam gscam -s 0 &
rosrun accompany_static_camera_localisation image_saver image:=/gscam/image_raw
