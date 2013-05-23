#include <Histogram.h>

#include <stdlib.h>
#include <iostream>
using namespace std;

#define TYPE_DATA  unsigned char
#define TYPE_WEIGHT double
#define DIM 2
#define BINS 3
#define MIN 0
#define MAX 255

int main()
{
  srand(time(0));
  HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX> histogram;

  cout<<"=== add some data"<<endl;
  vector<TYPE_DATA> data(DIM);
  for (int i=0;i<100;i++)
  {
    for (int j=0;j<DIM;j++)
    {
      data[j]=(TYPE_DATA)((((rand()%1000))/1000.0)*(MAX-MIN)+MIN);
      cout<<(double)(data[j])<<" ";
    }
    cout<<endl;
    histogram.add(data,(rand()/(TYPE_WEIGHT)RAND_MAX));
  }
  cout<<endl;
  
  cout<<"=== normalize"<<endl;
  TYPE_WEIGHT sum=0;
  vector<TYPE_WEIGHT> histo=histogram.normalize();
  for (unsigned i=0;i<histo.size();i++)
  {
    sum+=histo[i];
    cout<<histo[i]<<" ";
  }
  cout<<endl;
  cout<<"sum:"<<sum<<endl;
  cout<<endl;

  cout<<"=== add histogram"<<endl;
  HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX> hh=histogram+histogram;
  cout<<"histogram+histogram:"<<hh<<endl;
  cout<<"histogram:"<<histogram<<endl;
  cout<<endl;

  cout<<"=== test copy constructor"<<endl;
  for (int j=0;j<DIM;j++)
    data[j]=MAX/2;
  for (int i=0;i<100;i++)
    histogram.add(data,1); // add alot of data to original
  cout<<"should be effected by add:"<<endl;
  cout<<"  histogram:"<<histogram<<endl;
  cout<<"should be uneffected:"<<endl;
  cout<<"  histogram+histogram:"<<hh<<endl;
  cout<<endl;

}
