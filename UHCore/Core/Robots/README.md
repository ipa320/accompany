The classes in this directory handle the majority of the functionality of the UHCore module.
Provided are means of reading and updaing sensors, 
  controlling various robots, 
  generating views of sensors, 
  and recording sensor and robot activity.

Project directory is structured as follows:  
__Empathy:__ Scripts used during the empathy study  
__ROS_Packages:__ Manifests that can be loaded into ros at runtime by rosHelper  
__careobot.py:__ Implements the Robot interface for IPA Care-O-Bot models of robots  
__dummy.py:__ Implements the Robot interface with stub methods, used for testing  
__robot.py:__ Abstract base classes that define the robot interfaces  
__robotFactory.py:__ Provides methods of building an appropriate robot classes  
__rosHelper.py:__ Provides unified access into the ROS messaging system and handles all initialisation and connectivity  
__rosMulti.py:__ Experimental module that aims to provide the ability to connect to multiple ROS instances  
&nbsp;&nbsp; *Note*: This is functional for methods provided by the rosHelper interface, but does not yet full wrap rospy, and thus is not suited for use at this time  
__sunflower.py:__ Implements the Robot interface for the UH Sunflower models of robots  
