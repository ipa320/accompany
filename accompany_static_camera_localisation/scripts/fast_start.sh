export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.1.222:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=512, height=486, framerate=10/1 ! ffmpegcolorspace"

echo "build background model"

roslaunch accompany_static_camera_localisation capture_background_images.launch image_topic:=/gscam/image_raw nr_images:=10 res_path:=/home/robolab/ros/accompany/accompany_static_camera_localisation/res &
roslaunch accompany_static_camera_localisation build_background_model.launch res_path:=/home/robolab/ros/accompany/accompany_static_camera_localisation/res &
roslaunch accompany_static_camera_localisation fisheye_localization.launch
