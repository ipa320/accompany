export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.1.20:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=512, height=486, framerate=15/1 ! ffmpegcolorspace"
ROS_NAMESPACE=/camera_sofa rosrun gscam gscam -f camera_sofa gscam:=/camera_sofa -i /home/accompany/ros/accompany/UvA/Heerlen/cameras/camera_intrinsic_sofa.xml --sync false
