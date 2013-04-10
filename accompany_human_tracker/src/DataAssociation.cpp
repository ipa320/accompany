#include <DataAssociation.h>

#include <limits>
using namespace std;

DataAssociation::DataAssociation()
{
  clear(0,0);
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
  assign1.resize(size1);
  for (unsigned i=0;i<size1;i++)
    assign1[i]=-1;
  assign2.resize(size2);
  for (unsigned i=0;i<size2;i++)
    assign2[i]=-1;
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
  while (true)
  {
    std::pair<int,int> p=getMax(order);
    if (p.first<0)
      break;
    association[p.second]=p.first;
    cout<<"association["<<p.second<<"]="<<p.first<<endl;
  }
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

std::pair<int,int> DataAssociation::getMax(int order)
{
  std::pair<int,int> ret;
  ret.first=-1;
  ret.second=-1;
  double max=numeric_limits<double>::max()*-order;
  //cout<<"max:"<<max<<endl;
  //cout<<"size1:"<<size1<<" size2:"<<size2<<endl;
  for (unsigned i=0;i<size1;i++)
  {
    if (assign1[i]<0) // unassigned 1
    {
      for (unsigned j=0;j<size2;j++)
      {
        if (assign2[j]<0) // unassigned 2
        {
          //cout<<"i:"<<i<<" j:"<<j<<endl;
          if (associations[i][j]*order>max*order)
          {
            max=associations[i][j]*order;
            ret.first=i;
            ret.second=j;
          }
        }
      }
    }
  }
  //cout<<"ret.first:"<<ret.first<<endl;
  //cout<<"ret.second:"<<ret.second<<endl;
  if (ret.first>=0)
    assign1[ret.first]=ret.second;
  if (ret.second>=0)
    assign2[ret.second]=ret.first;
  return ret;
}
