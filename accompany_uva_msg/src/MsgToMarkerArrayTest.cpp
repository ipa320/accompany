
#include <accompany_uva_msg/MsgToMarkerArray.h>

#include <iostream>
using namespace std;

int main()
{

  MsgToMarkerArray msgToMarkerArray;
  std::string name="test";

  accompany_uva_msg::TrackedHumans msg;
  accompany_uva_msg::TrackedHuman h;
  h.location.header.frame_id="/myFrame";
  msg.trackedHumans.push_back(h);  

  visualization_msgs::MarkerArray &markerArray=msgToMarkerArray.toMarkerArray(msg,name);
  cout<<"================"<<markerArray<<endl;
  
  msg.trackedHumans.push_back(h);
  msg.trackedHumans.push_back(h);

  markerArray=msgToMarkerArray.toMarkerArray(msg,name);
  cout<<"================"<<markerArray<<endl;

  markerArray=msgToMarkerArray.toMarkerArray(msg,name);
  cout<<"================"<<markerArray<<endl;
  
  accompany_uva_msg::TrackedHumans msg2;
  msg2.trackedHumans.push_back(h);

  markerArray=msgToMarkerArray.toMarkerArray(msg2,name);
  cout<<"================"<<markerArray<<endl;

  msg2.trackedHumans.push_back(h);

  markerArray=msgToMarkerArray.toMarkerArray(msg2,name);
  cout<<"================"<<markerArray<<endl;

  accompany_uva_msg::TrackedHumans msg3;
  msg3.trackedHumans.push_back(h);
  
  markerArray=msgToMarkerArray.toMarkerArray(msg3,name);
  cout<<"================"<<markerArray<<endl;
  
  markerArray=msgToMarkerArray.toMarkerArray(msg3,name);
  cout<<"================"<<markerArray<<endl;
  

  return 0;
}
