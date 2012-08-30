
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt32.h>

#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp> 

#include <iostream>
#include <fstream> 
using namespace std;

template<class T> void save_msg(T &t,string filename)
{
  uint32_t size=ros::serialization::serializationLength(t);
  boost::shared_array<uint8_t> buffer(new uint8_t[size]);
  ros::serialization::OStream ostream(buffer.get(),size);
  ros::serialization::serialize(ostream,t);
  std::ofstream file(filename.c_str(),ios::out|ios::binary);
  for (unsigned int i=0;i<size;i++)
    file<<buffer[i];
  file.close();
}

template<class T> void load_msg(T &t,string filename)
{
  uint32_t size=ros::serialization::serializationLength(t);
  int byteSize=size*sizeof(uint32_t);
  unsigned char *buf=new unsigned char[byteSize];
  std::ifstream file(filename.c_str(),ios::in|ios::binary);
  file.readsome((char *)buf,byteSize);
  
  boost::shared_array<uint8_t> buffer(buf);
  ros::serialization::IStream istream(buffer.get(),size);
  ros::serialization::deserialize(istream,t);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_frame");
  
  geometry_msgs::PoseStamped p;
  p.pose.position.x=88;
  p.pose.orientation.w=1;
  save_msg(p,"test.dat");
  cout<<"p:"<<p;

  geometry_msgs::PoseStamped p2;
  load_msg(p2,"test.dat");
  cout<<"p2:"<<p2;

  
}
