#include <IDToName.h>

#include <stdexcept> 
using namespace std;

// static member init
const char IDToName::unkown[]="unkown";

void IDToName::setIDName(unsigned id,string name)
{
  try
  {
    string oldName=idToName.at(id);
    if (oldName.compare(name)!=0) // found old name
    {
      setIDNameHelper(id,name);
      nameToID.erase(oldName); // remove old
    }
  } 
  catch (const std::out_of_range& e) // no old name found
  {
    setIDNameHelper(id,name);
  }
}

string IDToName::getIDName(unsigned id)
{
  string name=IDToName::unkown;
  try
  {
    name=idToName.at(id);
  } 
  catch (const std::out_of_range& e)
  {
  }
  return name;
}

void IDToName::setIDNameHelper(unsigned id,std::string name)
{
  try
  {
    unsigned oldID=nameToID.at(name);
    idToName.erase(oldID); // if found old id
  } 
  catch (const std::out_of_range& e)
  {
  }
  idToName[id]=name;
  nameToID[name]=id;
}
