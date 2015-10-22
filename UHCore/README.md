University of Hertfordshire Adaptive System Group Python Library
---

This library provides access to read from and control some of the robots that are used in the UH ASRG robot lab.  
It provides provides generic methods for controlling multiple models of robots.  
Currently supported are *IPA Care-O-Bot (3.2, 3.5, 3.6)* and *UH Sunflower (1.1, 1.2)*

Project is structured as follows:  
__CppInterface:__ Provides C++ libraries for UHCore\Robot and UHCore\ActionHistory classes  
__CppInterfaceTest:__ Simple examples of how to use the above library  
__WebUI:__ Cherrypy based web interfaces.  Any web rendering code (including UI and API code) goes here.  
__Core:__ Primary repository for robot and data manipulation.  Most code lives here.  
