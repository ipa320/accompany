

# -------------------------
# ---  Introduction
# -------------------------

This document describes how to localise the Nomad robot

# -------------------------
# ---  Dependencies
# -------------------------

Install ros package 'Nomad'

  git clone git://basterwijn.nl/home/bterwijn/git/Nomad

Install ros package 'FreedomJoystick'

  git clone git://basterwijn.nl/home/bterwijn/git/FreedomJoystick

# -------------------------
# ---  Build map
# -------------------------

Connect:
- Nomad robot
- Hokuyo laser scanner 
- FreedomJoystick

Start the exploration:

  roslaunch explore.launch

Optionally start the visualisation:

  rosrun rviz rviz -d ./rviz.vcg

Drive the robot around in the environment using the joystick so that
all walls are detected by the laser.

Save the resulting map:

  rosrun map_server map_saver -f mymap

# -------------------------
# ---  Localise using map
# -------------------------

Connect:
- Nomad robot
- Hokuyo laser scanner 
- FreedomJoystick

Load the map:

  rosrun map_server map_server mymap.yaml

Start the visualisation:

  rosrun rviz rviz -d ./rviz.vcg

Start the localisation:

  roslaunch localise.launch

Optionally click "2D Pose Estimate" button in visualisation and
click-drag the correct position of the robot on the map.


