#include <DataAssociation.h>

using namespace std;

DataAssociation::DataAssociation()
{
}

void DataAssociation::clear(unsigned s1,unsigned s2)
{
  size1=s1;
  size2=s2;
  associations.resize(size1);
  for (unsigned i=0;i<size1;i++)
  {
    associations[i].resize(size2);
    for (unsigned j=0;j<size2;j++)
    {
      associations[i][j]=0;
    }
  }
}

void DataAssociation::set(unsigned d1,unsigned d2,double association)
{
  associations[d1][d2]=association;
}
  
std::vector<int> DataAssociation::associate(int order)
{
  std::vector<int> association(size2);
  for (unsigned i=0;i<size2;i++)
    association[i]=-1;
  return association;
}

std::ostream& operator<<(std::ostream& out,const DataAssociation& dataAssociation)
{
  out<<"DataAssociation:"<<endl;
  for (unsigned i=0;i<dataAssociation.size1;i++)
  {
    for (unsigned j=0;j<dataAssociation.size2;j++)
    {
      out<<dataAssociation.associations[i][j]<<" ";
    }
    out<<endl;
  }
  return out;
}
