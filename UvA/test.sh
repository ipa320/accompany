

# Test
# downloads prerecorded video and does detection and tracking
roscd accompany_uva/scripts
./startTestRobotHouse.sh

# view images
roslaunch accompany_uva trackRobotHouseViewImages.launch

# rviz
rosrun rviz rviz
# add tf,map,markerArray to display

