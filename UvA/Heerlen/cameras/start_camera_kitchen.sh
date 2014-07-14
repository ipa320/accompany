export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.1.21:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=512, height=486, framerate=15/1 ! ffmpegcolorspace"
ROS_NAMESPACE=/camera_kitchen rosrun gscam gscam -f camera_kitchen gscam:=/camera_kitchen -i /home/accompany/ros/accompany/UvA/Heerlen/cameras/camera_intrinsic_kitchen.xml --sync false
