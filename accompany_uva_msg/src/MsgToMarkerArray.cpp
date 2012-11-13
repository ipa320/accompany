
#include <accompany_uva_msg/MsgToMarkerArray.h>
#include <boost/functional/hash.hpp>

using namespace std;

visualization_msgs::MarkerArray MsgToMarkerArray::toMarkerArray(const accompany_uva_msg::HumanLocations& msg,
                                                                std::string name)
{
  visualization_msgs::MarkerArray markerArray;
  for (unsigned int i=0;i<msg.locations.size();i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg.locations[0].header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = name;
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg.locations[i].point.x;
    marker.pose.position.y = msg.locations[i].point.y;
    marker.pose.position.z = msg.locations[i].point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color=getRandomColor(msg.locations[0].header.frame_id,0.5);
    markerArray.markers.push_back(marker);
  }
  return markerArray;
}

visualization_msgs::MarkerArray MsgToMarkerArray::toMarkerArray(const accompany_uva_msg::TrackedHumans& msg,
                                                                std::string name)
{
  visualization_msgs::MarkerArray markerArray;
  for (unsigned int i=0;i<msg.trackedHumans.size();i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg.trackedHumans[i].location.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = name;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg.trackedHumans[i].location.point.x;
    marker.pose.position.y = msg.trackedHumans[i].location.point.y;
    marker.pose.position.z = msg.trackedHumans[i].location.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.2;
    markerArray.markers.push_back(marker);
  }
  return markerArray;
}

// --------------------------------
// --- generate pseudo random color using name
// --------------------------------

size_t MsgToMarkerArray::hashString(string data)
{
  size_t h=0;
  for (unsigned int i=0;i<data.size();i++)
  {
    int cut=h>>(sizeof(size_t)*7);
    h=((h<<7)^cut)^data[i];
  }
  return h;
}

size_t MsgToMarkerArray::reverseBits(size_t data)
{
  size_t ret=0;
  for (unsigned int i=0;i<sizeof(size_t)*8;i++)
  {
    if (((1<<i)&data)>0)
      ret|=1<<(sizeof(size_t)*8-i-1);
  }
  return ret;
}

// get a random color wich will be associated with the name on future invocations
std_msgs::ColorRGBA MsgToMarkerArray::getRandomColor(string name,double a)
{
  map<string,std_msgs::ColorRGBA>::const_iterator it=nameToColor.find(name);
  std_msgs::ColorRGBA color;
  if (it==nameToColor.end())
  {
    //    boost::hash<string> hasher;
    size_t hash=reverseBits(hashString(name));
    color.r=((hash>> 0)%255)/255.0; // trick to generate color from name
    color.g=((hash>> 8)%255)/255.0;
    color.b=((hash>>16)%255)/255.0;
    color.a=a;
    nameToColor.insert(pair<string,std_msgs::ColorRGBA>(name,color));
  }
  else
    color=it->second;
  return color;
}
