
#include <accompany_uva_msg/MsgToMarkerArray.h>
#include <boost/functional/hash.hpp>
#include <tf/transform_datatypes.h>

using namespace std;

visualization_msgs::MarkerArray &MsgToMarkerArray::toMarkerArray(const accompany_uva_msg::HumanLocations& msg,
                                                                 std::string name)
{
  visualization_msgs::MarkerArray &markerArray=getMarkerArray(name,msg.locations.size());
  double radius=0.05;
  for (unsigned int i=0;i<msg.locations.size();i++)
  {
    markerArray.markers[i].header.frame_id = msg.locations[0].header.frame_id;
    markerArray.markers[i].header.stamp = ros::Time();
    markerArray.markers[i].type = visualization_msgs::Marker::SPHERE;
    markerArray.markers[i].pose.position.x = msg.locations[i].point.x;
    markerArray.markers[i].pose.position.y = msg.locations[i].point.y;
    markerArray.markers[i].pose.position.z = msg.locations[i].point.z+radius;
    markerArray.markers[i].pose.orientation.x = 0.0;
    markerArray.markers[i].pose.orientation.y = 0.0;
    markerArray.markers[i].pose.orientation.z = 0.0;
    markerArray.markers[i].pose.orientation.w = 1.0;
    markerArray.markers[i].scale.x = radius*2;
    markerArray.markers[i].scale.y = radius*2;
    markerArray.markers[i].scale.z = radius*2;
    markerArray.markers[i].color=getColorByName(msg.locations[0].header.frame_id,0.5);
    markerArray.markers[i].lifetime = ros::Duration(10);
  }
  return markerArray;
}

visualization_msgs::MarkerArray &MsgToMarkerArray::toMarkerArray(const accompany_uva_msg::HumanDetections& msg,
                                                                 std::string name)
{
  visualization_msgs::MarkerArray &markerArray=getMarkerArray(name,msg.detections.size());
  double radius=0.05;
  for (unsigned int i=0;i<msg.detections.size();i++)
  {
    markerArray.markers[i].header.frame_id = msg.detections[0].location.header.frame_id;
    markerArray.markers[i].header.stamp = ros::Time();
    markerArray.markers[i].type = visualization_msgs::Marker::SPHERE;
    markerArray.markers[i].pose.position.x = msg.detections[i].location.point.x;
    markerArray.markers[i].pose.position.y = msg.detections[i].location.point.y;
    markerArray.markers[i].pose.position.z = msg.detections[i].location.point.z+radius;
    markerArray.markers[i].pose.orientation.x = 0.0;
    markerArray.markers[i].pose.orientation.y = 0.0;
    markerArray.markers[i].pose.orientation.z = 0.0;
    markerArray.markers[i].pose.orientation.w = 1.0;
    markerArray.markers[i].scale.x = radius*2;
    markerArray.markers[i].scale.y = radius*2;
    markerArray.markers[i].scale.z = radius*2;
    markerArray.markers[i].color=getColorByName(msg.detections[0].location.header.frame_id,0.5);
    markerArray.markers[i].lifetime = ros::Duration(10);
  }
  return markerArray;
}

visualization_msgs::MarkerArray &MsgToMarkerArray::toMarkerArray(const accompany_uva_msg::TrackedHumans& msg,
                                                                 std::string name)
{
  visualization_msgs::MarkerArray &markerArray=getMarkerArray(name,msg.trackedHumans.size()*3);
  int ind=0;
  // sphere
  double radius=0.2;
  for (unsigned int i=0;i<msg.trackedHumans.size();i++)
  {
    markerArray.markers[ind].header.frame_id = msg.trackedHumans[i].location.header.frame_id;
    markerArray.markers[ind].header.stamp = ros::Time();
    markerArray.markers[ind].type = visualization_msgs::Marker::SPHERE;
    markerArray.markers[ind].pose.position.x = msg.trackedHumans[i].location.point.x;
    markerArray.markers[ind].pose.position.y = msg.trackedHumans[i].location.point.y;
    markerArray.markers[ind].pose.position.z = msg.trackedHumans[i].location.point.z+radius;
    markerArray.markers[ind].pose.orientation.x = 0.0;
    markerArray.markers[ind].pose.orientation.y = 0.0;
    markerArray.markers[ind].pose.orientation.z = 0.0;
    markerArray.markers[ind].pose.orientation.w = 1.0;
    markerArray.markers[ind].scale.x = radius*2;
    markerArray.markers[ind].scale.y = radius*2;
    markerArray.markers[ind].scale.z = radius*2;
    markerArray.markers[ind].color.r = 1.0;
    markerArray.markers[ind].color.g = 0.0;
    markerArray.markers[ind].color.b = 0.0;
    markerArray.markers[ind].color.a = 0.2;
    markerArray.markers[ind].lifetime = ros::Duration(10);
    ind++;
  }
  // speed
  for (unsigned int i=0;i<msg.trackedHumans.size();i++)
  {
    markerArray.markers[ind].header.frame_id = msg.trackedHumans[i].location.header.frame_id;
    markerArray.markers[ind].header.stamp = ros::Time();
    markerArray.markers[ind].type = visualization_msgs::Marker::ARROW; 
    markerArray.markers[ind].pose.position.x = msg.trackedHumans[i].location.point.x;
    markerArray.markers[ind].pose.position.y = msg.trackedHumans[i].location.point.y;
    markerArray.markers[ind].pose.position.z = msg.trackedHumans[i].location.point.z+radius;
    double sx=msg.trackedHumans[i].speed.vector.x; // speed vector
    double sy=msg.trackedHumans[i].speed.vector.y; 
    double angle=atan2(sy,sx);
    double length=sqrt(sx*sx+sy*sy); 
    markerArray.markers[ind].pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);// yaw
    markerArray.markers[ind].scale.x = .5;
    markerArray.markers[ind].scale.y = .5;
    markerArray.markers[ind].scale.z = length;
    markerArray.markers[ind].color.r = 1.0;
    markerArray.markers[ind].color.g = 0.0;
    markerArray.markers[ind].color.b = 0.0;
    markerArray.markers[ind].color.a = 0.2;
    markerArray.markers[ind].lifetime = ros::Duration(10);
    ind++;
  }
  // text
  for (unsigned int i=0;i<msg.trackedHumans.size();i++)
  {
    markerArray.markers[ind].header.frame_id = msg.trackedHumans[i].location.header.frame_id;
    markerArray.markers[ind].header.stamp = ros::Time();
    markerArray.markers[ind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[ind].pose.position.x = msg.trackedHumans[i].location.point.x;
    markerArray.markers[ind].pose.position.y = msg.trackedHumans[i].location.point.y;
    markerArray.markers[ind].pose.position.z = msg.trackedHumans[i].location.point.z+radius*2;
    markerArray.markers[ind].pose.orientation.x = 0.0;
    markerArray.markers[ind].pose.orientation.y = 0.0;
    markerArray.markers[ind].pose.orientation.z = 0.0;
    markerArray.markers[ind].pose.orientation.w = 1.0;
    markerArray.markers[ind].scale.x = 0.2;
    markerArray.markers[ind].scale.y = 0.2;
    markerArray.markers[ind].scale.z = 0.2;
    markerArray.markers[ind].color.r = 1.0;
    markerArray.markers[ind].color.g = 0.0;
    markerArray.markers[ind].color.b = 0.0;
    markerArray.markers[ind].color.a = 0.2;
    stringstream ss;
    ss<<msg.trackedHumans[i].id;
    if (msg.trackedHumans[i].identity.size()>0)
      ss<<","<<msg.trackedHumans[i].identity;
    markerArray.markers[ind].text=ss.str();
    markerArray.markers[ind].lifetime = ros::Duration(10);
    ind++;
  }
  return markerArray;
}

visualization_msgs::MarkerArray &MsgToMarkerArray::getMarkerArray(std::string name,unsigned size)
{
  std::map<std::string,visualization_msgs::MarkerArray>::iterator it=nameToMarkerArray.find(name);
  if (it==nameToMarkerArray.end())
  {
    // create new MarkerArray
    visualization_msgs::MarkerArray markerArray;
    for (unsigned i=0;i<size;i++)
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
    // requested size in previous call
    unsigned prevSize=nameToSize[name];
    
    // reduce length after deleting in previous step
    unsigned reduce=prevSize;
    if (size>reduce)
      reduce=size;
    if (reduce<it->second.markers.size())
      it->second.markers.resize(reduce);

    // delete unused markers
    for (unsigned i=size;i<it->second.markers.size();i++)
      it->second.markers[i].action = visualization_msgs::Marker::DELETE;

    // add markers
    for (unsigned i=it->second.markers.size();i<size;i++)
    {
      visualization_msgs::Marker marker;
      marker.ns = name;
      marker.id = i;
      marker.action = visualization_msgs::Marker::ADD;
      it->second.markers.push_back(marker);
    }

    // reactivate previously deleted markers
    for (unsigned i=prevSize;i<size;i++)
      it->second.markers[i].action=visualization_msgs::Marker::ADD;
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
  for (unsigned i=0;i<data.size();i++)
  {
    int cut=h>>(sizeof(size_t)*7);
    h=((h<<7)^cut)^data[i];
  }
  return h;
}

size_t MsgToMarkerArray::reverseBits(size_t data)
{
  size_t ret=0;
  for (unsigned i=0;i<sizeof(size_t)*8;i++)
  {
    if (((1<<i)&data)>0)
      ret|=1<<(sizeof(size_t)*8-i-1);
  }
  return ret;
}

// get a random color wich will be associated with the name on future invocations
std_msgs::ColorRGBA MsgToMarkerArray::getColorByName(string name,double a)
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
