#ifndef MsgToMarkerArray_h_INCLUDED
#define MsgToMarkerArray_h_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <accompany_uva_msg/HumanLocations.h>
#include <accompany_uva_msg/TrackedHumans.h>

#include <map>

class MsgToMarkerArray
{
 public:

   visualization_msgs::MarkerArray toMarkerArray(const accompany_uva_msg::HumanLocations& msg,
                                                       std::string name="");
   visualization_msgs::MarkerArray toMarkerArray(const accompany_uva_msg::TrackedHumans& msg,
                                                       std::string name="");
  
 private:
   size_t hashString(std::string data);
   size_t reverseBits(size_t data);
   std_msgs::ColorRGBA getRandomColor(std::string name,double a=1.0);

  // members
   std::map<std::string,std_msgs::ColorRGBA> nameToColor;
};

#endif
