

# install Ubuntu11.10 (64bit), update all packages, reboot

#dirs
mkdir -p ~/programs
mkdir -p ~/ros

# init 
sudo apt-get -y install aptitude emacs git gitk mercurial libopencv-dev cmake gtk2-engines-pixbuf 

# gstreamer
sudo apt-get -y install gstreamer-tools gstreamer0.10-plugins-base gstreamer0.10-plugins-good gstreamer0.10-plugins-bad gstreamer0.10-plugins-ugly gstreamer0.10-ffmpeg libgstreamer0.10-dev 
libgstreamer-plugins-bad0.10-dev libgstreamer-plugins-base0.10-dev

# vxl for ubuntu >= 12.04
sudo apt-get -y install libvxl1-dev

cd ~/ros
git clone git://basterwijn.nl/home/bterwijn/git/accompany.git
cd

# ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install ros-groovy-desktop-full

sudo rosdep init
rosdep update
echo "" >> ~/.bashrc
echo "# ROS" >> ~/.bashrc
echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
echo "export ROS_WORKSPACE=~/ros" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_WORKSPACE:\$ROS_PACKAGE_PATH" >> ~/.bashrc
. ~/.bashrc
source /opt/ros/groovy/setup.bash
export ROS_WORKSPACE=~/ros
export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH

# vxl only applicable when Ubuntu < 12.04
# cd ~/programs
# wget http://downloads.sourceforge.net/project/vxl/vxl/1.17/vxl-1.17.0.zip
# unzip vxl-1.17.0.zip
# cd vxl-1.17.0/
# mkdir build
# cd build
# cmake .. -DBUILD_BRL=OFF
# make -j 4
# sudo make install

# cmnGwenn
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/cmnGwenn.git
cd cmnGwenn
mkdir build
cd build
cmake ../src
make
sudo make install

# yaml-cpp
cd ~/programs
hg clone https://code.google.com/p/yaml-cpp.new-api yaml-cpp
cd yaml-cpp
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# LogProbOp
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/LogProbOp.git
cd LogProbOp
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# GaussianMixture
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/GaussianMixture.git
cd GaussianMixture
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# KalmanFilter
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/KalmanFilter.git
cd KalmanFilter
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# cob_perception_common
cd ~/ros
git clone git://github.com/ipa320/cob_perception_common.git
rosdep install cob_perception_common
rosmake cob_perception_common

# cob_people_perception
cd ~/ros
git clone https://github.com/ipa320/cob_people_perception.git
rosdep install cob_people_perception
rosmake cob_people_perception

# gscam
rosdep install gscam
rosmake gscam

# map server
sudo apt-get -y install ros-groovy-navigation
rosdep install navigation
rosmake navigation

# rviz
sudo apt-get -y install ros-groovy-viz
rosdep install rviz
rosmake rviz

# rospy
#sudo apt-get -y install ros-groovy-rospy
rosdep install rospy
rosmake rospy

# rosbag
#sudo apt-get -y install ros-groovy-rosbag
rosdep install rosbag
rosmake rosbag

# accompany
rosdep install accompany_uva
rosmake accompany_uva


