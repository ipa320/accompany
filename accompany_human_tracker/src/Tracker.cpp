#include <Tracker.h>

using namespace std;

void Tracker::processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections)
{
  dataAssociation.clear(tracks.size(),humanDetections->detections.size());
  for (unsigned i=0;i<tracks.size();i++)
  {
    for (unsigned j=0;j<humanDetections->detections.size();j++)
    {
      dataAssociation.set(i,j,tracks[i].match(humanDetections->detections[j]));
    }
  }
  vector<int> associations=dataAssociation.associate();
  for (unsigned i=0;i<associations.size();i++)
  {
    if (associations[i]<0) // not assigned
    {
      tracks.push_back(Track(humanDetections->detections[i]));
    }
    else // assigned
    {
      tracks[associations[i]].match(humanDetections->detections[i]);
    }
  }
  
  cout<<*this<<endl;
}

std::ostream& operator<<(std::ostream& out,const Tracker& tracker)
{
  out<<"Tracker ("<<tracker.tracks.size()<<"):"<<endl;
  out<<tracker.dataAssociation;
  for (unsigned i=0;i<tracker.tracks.size();i++)
    out<<"["<<i<<"] "<<tracker.tracks[i];
  out<<endl;
  return out;
}
