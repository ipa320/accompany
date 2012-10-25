
./downloadTestData.sh
./reencodeTestVideo.sh

# ---------
# --- test video
#
export GSCAM_CONFIG="filesrc location=$PWD/../testData/testVideo.flv ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=640, height=480, framerate=50/1 ! ffmpegcolorspace"


# ---------
# --- live camera
#
#export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=640, height=480, framerate=15/1 ! ffmpegcolorspace"


roslaunch ./launch/test.launch
