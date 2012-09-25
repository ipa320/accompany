rosbag record -b 0 /gscam/image_raw/theora
#
# to play back use image_transport to convert from theora to raw images like so:
#
#
# rosbag play -l -d 5 <bag-file>
# rosrun image_transport republish theora in:=/gscam/image_raw _image_transport:=theora raw out:=/gscam/image_raw
# rosrun image_view image_view image:=/gscam/image_raw
#
