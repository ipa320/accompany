export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@10.0.1.161:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=512, height=486, framerate=15/1 ! ffmpegcolorspace"
ROS_NAMESPACE=/camera_sofa rosrun gscam gscam -f camera_sofa gscam:=/camera_sofa -i /home/accompagny/ros/accompany/UvA/Troyes/cameras/camera_intrinsic_sofa.xml --sync false
