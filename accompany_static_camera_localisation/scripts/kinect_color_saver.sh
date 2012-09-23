roscore &
rosrun openni_camera openni_node &
rosrun image_view image_view image:=/camera/rgb/image_color &
rosrun accompany_static_camera_localisation image_saver image:=/camera/rgb/image_color
