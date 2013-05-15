#ifndef IDToName_INCLUDED
#define IDToName_INCLUDED

#include <map>
#include <string>

class IDToName
{
 public:
  static const char unkown[];

  void setIDName(unsigned id,std::string name);
  std::string getIDName(unsigned id);

 private:

  std::map<unsigned,std::string> idToName; // map of id's to names
  std::map<std::string,unsigned> nameToID; // map of names to id's

  void setIDNameHelper(unsigned id,std::string name);

};

#endif
