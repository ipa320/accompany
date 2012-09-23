roscd accompany_static_camera_localisation/res/

READ IMAGES
mkdir -p new_background_images

MOVE IMAGES
mkdir -p background_images

roslaunch accompany_static_camera_localisation create_background_model.launch
