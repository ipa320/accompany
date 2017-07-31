#!/bin/bash
rosrun rosbag record \
    -o sofa \
    --topic /camera_sofa/camera_info \
            /camera_sofa/image_raw/theora \
            /camera_sofa/image_raw/compressed

