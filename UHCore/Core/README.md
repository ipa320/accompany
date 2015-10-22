The classes in this directory handle the majority of the functionality of the UHCore module.
Provided are means of reading and updaing sensors, 
  controlling various robots, 
  generating views of sensors, 
  and recording sensor and robot activity.

Project directory is structured as follows:  
__Data:__ Manages reading manipulating and writing of data  
__Lib:__ External libraries included in the project  
__Robots:__ Monitoring and control of robots  
__SensorMap:__ Building images of sensors (active and historical)  
__config.py:__ Central storage of any magic strings/numbers  
__extensions.py:__ Theading extensions used by various classes  
__hitory.py:__ Updating the database for changes in sensors and robot actions  
__locations.py:__ Reading location data for tracked people/robots  
&nbsp;&nbsp; *Note*: Executing this fill will begin automatic updates for people/robot locations  
__sensors.py:__ Reading sensor data an converting to a common format  
&nbsp;&nbsp; *Note*: Executing this file will begine reading and recording sensor data  
