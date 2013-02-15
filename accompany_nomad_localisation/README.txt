
# -------------------------
# ---  Introduction
# -------------------------

This document describes how to first build a map of the environment
and then localise the Nomad robot based on the fixed coordinate system
of the map.

See 'NomadLocalisation.odg' for an overview of the data flow between
the components involved.

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
click-drag the correct position of the robot on the visualized map.

Now when driving the robot around the transform from /map to
/base_link gives the location of the robot within the map.
