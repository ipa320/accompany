#include <DataAssociation.h>

#include <limits>
using namespace std;

/**
 * Constructor
 */
DataAssociation::DataAssociation()
{
  clear(0,0);
}

/**
 * Allocate for matching e1 against e2 entities
 * @param e1 number of entities
 * @param e2 number of entities
 */
void DataAssociation::clear(unsigned e1,unsigned e2)
{
  size1=e1;
  size2=e2;
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

/**
 * Set the association value for entity e1 and entity e2
 * @param e1 number of entities
 * @param e2 number of entities
 * @param association association value
 */
void DataAssociation::set(unsigned e1,unsigned e2,double association)
{
  associations[e1][e2]=association;
}
  
/**
 * Associate entities after all association values are set
 * @param threshold threshold of the association values
 * @param order 1 if higher association values are better -1 if lower association are better
 * @returns vector of length e2 which holds indexes to the associated e1 entities or -1 if it could not be associated
 */
std::vector<int> DataAssociation::associate(double threshold,int order)
{
  cout<<"associate "<<size1<<" tracks with "<<size2<<" detections"<<endl;
  std::vector<int> association(size2);
  for (unsigned i=0;i<size2;i++)
    association[i]=-1;
  while (true)
  {
    std::pair<int,int> p=getMax(threshold,order);
    if (p.first<0)
      break;
    association[p.second]=p.first;
    cout<<"association["<<p.second<<"]="<<p.first<<endl;
  }
  return association;
}

/**
 * ostreams DataAssociation
 */
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

/**
 * Get the next association based on the best remaining association value
 * @param threshold threshold of the association values
 * @param order 1 if higher association values are better -1 if lower association are better
 */
std::pair<int,int> DataAssociation::getMax(double threshold,int order)
{
  std::pair<int,int> ret;
  ret.first=-1;
  ret.second=-1;
  double max=(numeric_limits<double>::max()/2)*-order;
  for (unsigned i=0;i<size1;i++)
  {
    if (assign1[i]<0) // unassigned 1
    {
      for (unsigned j=0;j<size2;j++)
      {
        if (assign2[j]<0) // unassigned 2
        {
          if (associations[i][j]*order>threshold*order &&
              associations[i][j]*order>max*order)
          {
            max=associations[i][j]*order;
            ret.first=i;
            ret.second=j;
          }
        }
      }
    }
  }
  if (ret.first>=0)
    assign1[ret.first]=ret.second;
  if (ret.second>=0)
    assign2[ret.second]=ret.first;
  return ret;
}
