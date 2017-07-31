#!/bin/bash
rosrun rosbag record \
    -o kitchen \
    --topic /camera_kitchen/camera_info \
            /camera_kitchen/image_raw/theora \
            /camera_kitchen/image_raw/compressed
