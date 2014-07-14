
ACCOMPANY (ANY-150) Dataset
===========================
Instruction for recording


Database ---------- 
# check if timestamp is in sync with ros time 
# the command below on the computer with the database lists the last
# 20 sensor events, trigger a sensor and make a note of the time
# offset to ros time so we can correct it later.

# additionally you might want to make a note of the sensorIds of the
# sensors of interest so we know which is which for sure

mysql -u accompanyUser -paccompany -e "use Accompany;select * from Accompany.SensorLog order by timestamp desc limit 20;"

# use this at the end of all recordings to create a dump of the entire
# database that holds all sensor data

mysqldump -u accompanyUser -paccompany Accompany > UH_database.sql


Start
----------

# start roscore in a single system:

roscore

# on all other machines set the ros_master_uri to point to that system
# for this to work each computer must be able to ping all other 
# computers therefore if nessecary add ip for each hostname to /etc/hosts 

export ROS_MASTER_URI=http://hostname:11311/

# make sure the real time is used

rosparam set use_sim_time false

# possibly start each start_* script on separate computers for more
# resources



Record
----------

# start record scripts, load balance over mutiple computers, harddisk
# write speed is the first bottleneck

# to view contents after recording
rosbag info <bag-file>



Playback
----------

# use sim time

rosparam set use_sim_time true

# play back all bag files

rosbag play --clock <multiple-bag-files>



Reconstruct
----------

# reconstruct the raw image data from the recorded compressed data and
# display them

roslaunch reconstruct_data.launch

# check frame rate, this should give the frame rate of a topic that
# should be roughly equal to the frame rate of the original
# stream. Better check the end of a recording as disk are cached so
# the start can look good but at the end frames could still have been
# dropped.

rostopic hz <topic-name>

