export GSCAM_CONFIG="rtspsrc location=rtsp://admin:sadmin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"

roslaunch accompany_static_camera_localisation viewImages.launch
