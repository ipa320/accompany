
#include <ros/ros.h>
#include <std_msgs/UInt32.h>

#include <iostream>
#include <fstream>

template<class T> void save_msg(T &t,std::string filename)
{
  uint32_t size=ros::serialization::serializationLength(t);
  boost::shared_array<uint8_t> buffer(new uint8_t[size]);
  ros::serialization::OStream ostream(buffer.get(),size);
  ros::serialization::serialize(ostream,t);
  std::ofstream file(filename.c_str(),std::ios::out|std::ios::binary);
  for (unsigned int i=0;i<size;i++)
    file<<buffer[i];
  file.close();
}

template<class T> bool load_msg(T &t,std::string filename)
{
  bool ret=false;
  std::ifstream file(filename.c_str(),std::ios::in|std::ios::binary);
  if(!file.fail())
  {
    file.seekg (0, std::ios::end);
    int length = file.tellg();// get length of file
    file.seekg (0, std::ios::beg);
    unsigned char *buf=new unsigned char[length];
    file.readsome((char *)buf,length);// read hole file
    file.close();
    boost::shared_array<uint8_t> buffer(buf);
    ros::serialization::IStream istream(buffer.get(),length/sizeof(uint8_t));
    ros::serialization::deserialize(istream,t);
    ret=true;
  }
  return ret;
}
