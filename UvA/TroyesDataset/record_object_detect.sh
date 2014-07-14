#!/bin/bash
rosrun rosbag record \
    -o object_detect \
    --topic /openni2_camera/parameter_descriptions \
            /openni2_camera/parameter_updates \
            /rgb/camera_info \
            /rgb/image/compressed \
            /depth/camera_info \
            /depth/image/compressedDepth
