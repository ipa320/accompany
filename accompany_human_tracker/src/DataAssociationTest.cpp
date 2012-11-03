
#include <DataAssociation.h>

#include <stdlib.h>
#include <vector>
#include <iostream>
using namespace std;

#define size1 3
#define size2 2
#define dim 1

struct VectorTraits 
{
  static unsigned int getSize(const vector< vector<double> >& v)
  {
    return v.size();
  }

  static const vector<double> &getElement(const vector< vector<double> >& v,int i)
  {
    return v[i];
  }
};

struct DistanceVector
{
  static double squareDistance(const vector<double>& v1,
                               const vector<double>& v2)
  {
    double sd=0;
    for (unsigned int i=0;i<v1.size();i++)
    {
      double d=v1[i]-v2[i];
      sd+=d*d;
    }
    return sd;
  }
};

std::ostream& operator<< (std::ostream& stream, const vector<double>& v)
{
  for (vector<double>::const_iterator it=v.begin();
       it!=v.end();
       it++)
    cout<<*it<<" ";
  return stream;
}

int main(int argc,char **argv)
{
  srand(time(0));

  vector< vector<double> > d1; // init d1
  for (int i=0;i<size1;i++)
  {
    vector<double> v;
    for (int d=0;d<dim;d++)
      v.push_back(rand()/(double)RAND_MAX);
    d1.push_back(v);
  }
  
  vector< vector<double> > d2; // init d2
  for (int i=0;i<size2;i++)
  {
    vector<double> v;
    for (int d=0;d<dim;d++)
      v.push_back(rand()/(double)RAND_MAX);
    d2.push_back(v);
  }

  // print data
  cout<<"d1:"<<endl;
  for (unsigned int i=0;i<d1.size();i++)
    cout<<d1[i]<<endl;
  cout<<"d2:"<<endl;
  for (unsigned int i=0;i<d2.size();i++)
    cout<<d2[i]<<endl;
  cout<<endl;

  DataAssociation< vector< vector<double> >,
                   VectorTraits,
                   vector< vector<double> >,
                   VectorTraits,
                   DistanceVector> dataAssociation;
  dataAssociation.buildDistanceMatrix(d1,d2);

  // print matrix
  cout<<"distance matrix:"<<endl;
  for (int i1=0;i1<size1;i1++)
  {
    for (int i2=0;i2<size2;i2++)
      cout<<dataAssociation.squareDistance(i1,i2)<<" ";
    cout<<endl;
  }
  cout<<endl;

  // get matches
  cout<<"best matches:"<<endl;
  double distance;
  int index1,index2;
  double sumDist=0;
  while (dataAssociation.bestMatch(distance,index1,index2))
  {
    cout<<distance<<" index1:"<<index1<<" index2:"<<index2<<endl;
    sumDist+=distance;
  }
  cout<<"sumDist: "<<sumDist<<endl;
  cout<<endl;

  cout<<"best global matches:"<<endl;
  double sumDistance;
  vector<std::pair<int,int> > indices=dataAssociation.globalBestMatches(sumDistance);
  for (unsigned int i=0;i<indices.size();i++)
    cout<<indices[i].first<<" - "<<indices[i].second<<endl;
  cout<<"sumDistance:"<<sumDistance<<endl;

  return 0;
}
