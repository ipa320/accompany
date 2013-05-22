#ifndef MsgToMarkerArray_h_INCLUDED
#define MsgToMarkerArray_h_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <accompany_uva_msg/HumanLocations.h>
#include <accompany_uva_msg/HumanDetections.h>
#include <accompany_uva_msg/TrackedHumans.h>

#include <map>

class MsgToMarkerArray
{
 public:

   visualization_msgs::MarkerArray &toMarkerArray(const accompany_uva_msg::HumanLocations& msg,
                                                  std::string name="");
   visualization_msgs::MarkerArray &toMarkerArray(const accompany_uva_msg::HumanDetections& msg,
                                                  std::string name="");
   visualization_msgs::MarkerArray &toMarkerArray(const accompany_uva_msg::TrackedHumans& msg,
                                                  std::string name="");
  
 private:
   size_t hashString(std::string data);
   size_t reverseBits(size_t data);
   std_msgs::ColorRGBA getColorByName(std::string name,double a=1.0);

   // removes the markers that are not used compared to previous call with same name
   //int removeUnusedMarkers(std::string name,visualization_msgs::MarkerArray &markerArra);

   visualization_msgs::MarkerArray &getMarkerArray(std::string name,unsigned size);

  // members
   std::map<std::string,std_msgs::ColorRGBA> nameToColor;
   std::map<std::string,visualization_msgs::MarkerArray> nameToMarkerArray;
   std::map<std::string,int> nameToSize;
};

#endif
