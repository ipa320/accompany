#ifndef MsgToMarkerArray_h_INCLUDED
#define MsgToMarkerArray_h_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <accompany_uva_msg/HumanLocations.h>


std_msgs::ColorRGBA getRandomColor(std::string name,double a=1.0);

visualization_msgs::MarkerArray toMarkerArray(const accompany_uva_msg::HumanLocations& msg,
                                              std::string name="",int id=0);

#endif
