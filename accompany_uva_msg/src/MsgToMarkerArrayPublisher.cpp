

#include <ros/ros.h>
#include <accompany_uva_msg/MsgToMarkerArray.h>

#include <string>
#include <iostream>

template<class Msg>
class MsgToMarkerArrayPublisher
{
public:

  typedef typename Msg::ConstPtr MsgConstPtr;

  MsgToMarkerArrayPublisher(ros::NodeHandle &n,
                            std::string topicIn,
                            std::string markerNameSpace,
                            std::string topicOut="visualization_marker_array")
  {    
    msgSub=n.subscribe<Msg>(topicIn,10,&MsgToMarkerArrayPublisher::receiveMsg,this);
    markerArrayPub=n.advertise<visualization_msgs::MarkerArray>(topicOut,0);
    this->markerNameSpace=markerNameSpace;
  }

  void receiveMsg(const MsgConstPtr& msg)
  {
    std::cout<<"receiveMsg:"<<*msg<<std::endl;
    markerArrayPub.publish(msgToMarkerArray.toMarkerArray(*msg,markerNameSpace)); // publish visualisation
  }

private:

  ros::Subscriber msgSub;
  ros::Publisher markerArrayPub;
  std::string markerNameSpace;
  MsgToMarkerArray msgToMarkerArray;
};

int main(int argc,char **argv)
{
  ros::init(argc, argv, "MsgToMarkerArrayPublisher");
  ros::NodeHandle n;
  
  MsgToMarkerArrayPublisher<accompany_uva_msg::TrackedHumans> msgToMarkerArrayPublisher(n,
                                                                                        "/trackedHumans",
                                                                                        "trackedHumans");
  ros::spin();

  return 0;
}

