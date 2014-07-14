export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"

mkdir -p ~/ros/accompany/accompany_static_camera_localisation/res/marker

roslaunch accompany_static_camera_localisation fisheye_marker_saver.launch
