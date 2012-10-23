export GSCAM_CONFIG="rtspsrc location=rtsp://admin:sadmin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"

mkdir -p ~/ros/accompany/accompany_static_camera_localisation/res/new_background_images

mkdir -p ~/ros/accompany/accompany_static_camera_localisation/res/background_images

roslaunch accompany_static_camera_localisation fisheye_capture_background_images.launch
