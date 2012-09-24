echo "READ IMAGES"

echo "MOVE IMAGES"

mkdir -p ~/ros/accompany/accompany_static_camera_localisation/res/new_background_images

mkdir -p ~/ros/accompany/accompany_static_camera_localisation/res/background_images

rosrun accompany_static_camera_localisation create_background_list ~/ros/accompany/accompany_static_camera_localisation/res/background_images/background_list.txt ~/ros/accompany/accompany_static_camera_localisation/res/background_images/*.jpg

roslaunch accompany_static_camera_localisation create_background_model.launch
