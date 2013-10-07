roscore &
roscd accompany_static_camera_localisation/res/ &
rosrun openni_camera openni_node &
rosrun camera_calibration cameracalibrator.py --size 6x8 --square 0.5 image:=/camera/rgb/image_color camera_info:=/camera
