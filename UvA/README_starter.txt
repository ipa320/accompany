#                         #
# UNIVERSITY OF AMSTERDAM #
#    ACCOMPANT PROJECT    #
#           2012          #
#                         #


# -------------------------
# ---  Introduction
# -------------------------

This document is a manual of using the UvA modules (DoW T4.1) of the ACCOMPANY project. We assume using the operation system of Ubuntu 11.10 (Oneiric) in the Robot House (UH).

----------------------------------------

#-- Capture new background images --#

The software contrains a training process of the background model. To make the background model more robust to the changes, the lighting conditions for example, the background images are recommanded to be captured everytime before starting the localization. 

First go to the folder, which contains scripts to capture background images:

  roscd accompany_static_camera_localisation/scripts/ 

FISHEYE:

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"

  ./fisheye_capture_background_images.sh

KINECT:

  ./kinect_capture_background_images.sh

Right click the image a few times whenever you think it is showing the background and there is no person appears in the frame. Each time that you click will copy the clicked frame to "accompany_static_camera_localisation/res/new_background_images".

You can view or remove the background images that are stored by going to:

  roscd accompany_static_camera_localisation/res/new_background_images

Next, these frames will be copied to "accompany_static_camera_localisation/res/background_images" and trained to generate a new background model.

If some frames containing foreground objects are added by mistake, you can remove them in:

  accompany_static_camera_localisation/res/background_images

----------------------------------------

#-- Build the background model --#

After checking all the background images are correct, you can invoke the script to build the background model:

  roscd accompany_static_camera_localisation/scripts/ 
  ./create_background_model.sh

Wait until the module is finished cleanly ("clean exit" is displayed in the command line). 

----------------------------------------

#-- Localization --#

FISHEYE:

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"

  roslaunch accompany_static_camera_localisation fisheye_localization.launch

KINECT:
   
  roslaunch accompany_static_camera_localisation kinect_localization.launch

Echo human localizations:

  rostopic echo /humanLocations
  
----------------------------------------

#-- Tuning parameters of template--# 

We use a Polyhedron template to model the person in a 3D space. Therefore if the default template does not fit the person (either too big or too small), you can change the template parameters:

  gedit accompany_static_camera_localisation/res/params.xml
  
You can change the following:

  <personHeight>1800</personHeight>:  height of the person
  <wg>400</wg>                     :  distance between the feet
  <wm>300</wm>                     :  width of the shoulders
  <wt>100</wt>                     :  width of the head
  <midRatio>.8</midRatio>          :  height of shoulders, in the percentage of the person's height
  <persDist>500</persDist>         :  the minimal distance between two persons
  
----------------------------------------

#-- Create Prior --#

Select a walkable region:

  roscd accompany_static_camera_localisation/res

  rosrun accompany_static_camera_localisation create_prior -l background_images/background_list.txt -p params.xml -o prior.txt -i camera_intrinsic.xml -e camera_extrinsic.xml
  
----------------------------------------

#-- Check calibration --#

Check the calibration results:

  roscd accompany_static_camera_localisation/res

  rosrun accompany_static_camera_localisation annotate_pos -l background_images/background_list.txt -p params.xml -r prior.txt -i camera_intrinsic.xml -e camera_extrinsic.xml -a temp.txt
  
#-- capture all the data --#

roslaunch accompanyroslaunch accompany_uva save_all_results_test.launch
