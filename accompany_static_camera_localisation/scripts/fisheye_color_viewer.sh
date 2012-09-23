roscore &
rosrun gscam gscam -s 0 &
rosrun image_view image_view image:=/gscam/image_raw &
rosrun accompany_static_camera_localisation image_saver image:=/gscam/image_raw
