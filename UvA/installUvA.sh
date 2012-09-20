echo "install ros electric"

echo "install vxl-1.17.0"
#wget http://downloads.sourceforge.net/project/vxl/vxl/1.17/vxl-1.17.0.zip?r=http%3A%2F%2Fsourceforge.net%2Fprojects%2Fvxl%2Ffiles%2Fvxl%2F1.17%2Fvxl-1.17.0.zip%2Fdownload&ts=1348135175&use_mirror=ignum

sudo apt-get -y install ubuntu-restricted-extras

sudo apt-get -y install aptitude emacs git libopencv2.3-dev cmake libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev gstreamer-tools gstreamer0.10-x ros-electric-openni-kinect

cd
mkdir -p ros
export ROS_WORKSPACE=~/ros
export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH
echo "export ROS_WORKSPACE=~/ros" >> ~/.bashrc
echo "ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH" >> ~/.bashrc
cd ros

# accompany
git clone git://basterwijn.nl/home/bterwijn/git/accompany.git

cd ~/
mkdir -p programs


# cmnGwenn
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/cmnGwenn.git
cd cmnGwenn
mkdir build
cd build
cmake ../src
make
sudo make install

# TimTracker
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/TimTracker.git
cd TimTracker
mkdir build
cd build
cmake ..
make
sudo make install

# gscam
cd ~/ros/accompany/UvA/dependencies/gscam
rosdep install gscam
rosmake

# skeleton_marker
svn checkout http://pi-robot-ros-pkg.googlecode.com/svn/trunk/skeleton_markers
rosdep install skeleton_markers
rosmake skeleton_markers

# cob_perception_common
cd ~/ros
git clone git://github.com/ipa320/cob_perception_common.git
rosdep install cob_perception_common
rosmake cob_perception_common

# cob_people_perception
cd ~/ros
git clone https://github.com/ipa320/cob_people_perception.git
# add these lines to cob_people_detection/CMakeLists.txt:
# target_link_libraries(face_recognizer_node boost_filesystem boost_system)
# target_link_libraries(detection_tracker_node boost_signals)
# target_link_libraries(people_detection_display_node boost_signals)
# target_link_libraries(face_capture_node boost_signals boost_filesystem boost_system)
rosdep install cob_people_perception
rosmake cob_people_detection

# UvA localization
rosdep install accompany_static_camera_localisation
rosmake accompany_static_camera_localisation

# Test
# downloads prerecorded video and does detection and tracking
roscd accompany/UvA/startScripts/
./startTestNonGSCam

# tracks humans and identities using artificial data
roslaunch accompany_human_tracker testTracker.launch
