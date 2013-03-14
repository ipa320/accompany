#include <Histogram.h>

#include <stdlib.h>
#include <iostream>
using namespace std;

#define TYPE_DATA float
#define TYPE_WEIGHT double
#define DIM 3
#define BINS 4
#define MIN 0
#define MAX 10

int main()
{
  srand(time(0));
  HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX> histogram;

  vector<TYPE_DATA> data(DIM);
  for (int i=0;i<100;i++)
  {
    for (int j=0;j<DIM;j++)
    {
      data[j]=((TYPE_DATA)((rand()%(MAX*100))))/100;
      cout<<data[j]<<" ";
    }
    cout<<endl;
    histogram.add(data,(rand()/(TYPE_WEIGHT)RAND_MAX));
  }
  
  TYPE_WEIGHT sum=0;
  vector<TYPE_WEIGHT> histo=histogram.normalize();
  for (unsigned i=0;i<histo.size();i++)
  {
    sum+=histo[i];
    cout<<histo[i]<<" ";
  }
  cout<<endl;
  cout<<"sum:"<<sum<<endl;

}

