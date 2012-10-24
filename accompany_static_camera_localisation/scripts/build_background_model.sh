#!/bin/bash

image_topic=""
nr_images=""
res_path=""
 
while getopts "t:n:p:" opt; do
  case $opt in
    t)
      image_topic=$OPTARG
      ;;
    n)
      nr_images=$OPTARG
      ;;
    p)
      res_path=$OPTARG
      ;;
    \?)
      echo "usage: $0 [-t <image-topic-name>] [-n <nr-images>] [-p <res-path>]" >&2
      exit 1
      ;;
    :)
      echo "usage: $0 [-t <image-topic-name>] [-n <nr-images>] [-p <res-path>]" >&2
      exit 1
      ;;
  esac
done

echo "image_topic:=$image_topic"
echo "nr_images:=$nr_images"
echo "res_path:=$res_path"

arguments=""
if [ "$image_topic" != "" ] 
then
    arguments="$arguments image_topic:=$image_topic"
fi
if [ "$nr_images" != "" ]
then
    arguments="$arguments nr_images:=$nr_images"
fi
if [ "$res_path" != "" ]
then
    arguments="$arguments res_path:=$res_path"
    #echo "mkdir -p $res_path/background_images"
    #mkdir -p $res_path/background_images
fi
echo "roslaunch accompany_static_camera_localisation capture_background_images.launch $arguments"
roslaunch accompany_static_camera_localisation capture_background_images.launch $arguments

arguments=""
if [ "$res_path" != "" ]
then
    arguments="$arguments res_path:=$res_path"
fi
echo "roslaunch accompany_static_camera_localisation build_background_model.launch $arguments"
roslaunch accompany_static_camera_localisation build_background_model.launch $arguments

