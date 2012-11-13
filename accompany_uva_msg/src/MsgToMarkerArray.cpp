
#include <accompany_uva_msg/MsgToMarkerArray.h>
#include <boost/functional/hash.hpp>

using namespace std;

visualization_msgs::MarkerArray &MsgToMarkerArray::toMarkerArray(const accompany_uva_msg::HumanLocations& msg,
                                                                 std::string name)
{
  visualization_msgs::MarkerArray &markerArray=getMarkerArray(name,msg.locations.size());
  for (unsigned int i=0;i<msg.locations.size();i++)
  {
    markerArray.markers[i].header.frame_id = msg.locations[0].header.frame_id;
    markerArray.markers[i].header.stamp = ros::Time();
    markerArray.markers[i].type = visualization_msgs::Marker::SPHERE;
    markerArray.markers[i].pose.position.x = msg.locations[i].point.x;
    markerArray.markers[i].pose.position.y = msg.locations[i].point.y;
    markerArray.markers[i].pose.position.z = msg.locations[i].point.z;
    markerArray.markers[i].pose.orientation.x = 0.0;
    markerArray.markers[i].pose.orientation.y = 0.0;
    markerArray.markers[i].pose.orientation.z = 0.0;
    markerArray.markers[i].pose.orientation.w = 1.0;
    markerArray.markers[i].scale.x = 0.1;
    markerArray.markers[i].scale.y = 0.1;
    markerArray.markers[i].scale.z = 0.1;
    markerArray.markers[i].color=getRandomColor(msg.locations[0].header.frame_id,0.5);
  }
  return markerArray;
}

visualization_msgs::MarkerArray &MsgToMarkerArray::toMarkerArray(const accompany_uva_msg::TrackedHumans& msg,
                                                                 std::string name)
{
  visualization_msgs::MarkerArray &markerArray=getMarkerArray(name,msg.trackedHumans.size());
  for (unsigned int i=0;i<msg.trackedHumans.size();i++)
  {
    markerArray.markers[i].header.frame_id = msg.trackedHumans[i].location.header.frame_id;
    markerArray.markers[i].header.stamp = ros::Time();
    markerArray.markers[i].type = visualization_msgs::Marker::SPHERE;
    markerArray.markers[i].pose.position.x = msg.trackedHumans[i].location.point.x;
    markerArray.markers[i].pose.position.y = msg.trackedHumans[i].location.point.y;
    markerArray.markers[i].pose.position.z = msg.trackedHumans[i].location.point.z;
    markerArray.markers[i].pose.orientation.x = 0.0;
    markerArray.markers[i].pose.orientation.y = 0.0;
    markerArray.markers[i].pose.orientation.z = 0.0;
    markerArray.markers[i].pose.orientation.w = 1.0;
    markerArray.markers[i].scale.x = 0.4;
    markerArray.markers[i].scale.y = 0.4;
    markerArray.markers[i].scale.z = 0.4;
    markerArray.markers[i].color.r = 1.0;
    markerArray.markers[i].color.g = 0.0;
    markerArray.markers[i].color.b = 0.0;
    markerArray.markers[i].color.a = 0.2;
  }
  return markerArray;
}

visualization_msgs::MarkerArray &MsgToMarkerArray::getMarkerArray(std::string name,int size)
{
  std::map<std::string,visualization_msgs::MarkerArray>::iterator it=nameToMarkerArray.find(name);
  if (it==nameToMarkerArray.end())
  {
    // create new MarkerArray
    visualization_msgs::MarkerArray markerArray;
    for (int i=0;i<size;i++)
    {
      visualization_msgs::Marker marker;
      marker.ns = name;
      marker.id = i;
      marker.action = visualization_msgs::Marker::ADD;
      markerArray.markers.push_back(marker);
    }
    it=nameToMarkerArray.insert(pair<std::string,visualization_msgs::MarkerArray>(name,markerArray)).first;
  }
  else
  {    
    int prevSize=nameToSize[name];
    
    // reduce length after deleting in previous step
    if (prevSize<size)
      prevSize=size;
    if (prevSize<it->second.markers.size())
      it->second.markers.resize(prevSize);

    // delete unused markers
    for (unsigned int i=size;i<it->second.markers.size();i++)
      it->second.markers[i].action = visualization_msgs::Marker::DELETE;

    // add markers
    for (unsigned int i=it->second.markers.size();i<size;i++)
    {
      visualization_msgs::Marker marker;
      marker.ns = name;
      marker.id = i;
      marker.action = visualization_msgs::Marker::ADD;
      it->second.markers.push_back(marker);
    }
  }
  nameToSize[name]=size;
  return it->second;
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
