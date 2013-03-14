#include <Histogram.h>

#include <stdlib.h>
#include <iostream>
using namespace std;

#define TYPE double
#define DIM 3
#define BINS 4
#define MIN 0
#define MAX 10

int main()
{
  srand(time(0));
  HistogramInt<TYPE,BINS,DIM,MIN,MAX> histogram;

  vector<TYPE> data(DIM);
  for (int i=0;i<100;i++)
  {
    for (int j=0;j<DIM;j++)
    {
      data[j]=(rand()%(MAX*100))/100;
      cout<<data[j]<<",";
    }
    cout<<endl;
    histogram.add(data);
  }
  
  vector<float> histo=histogram.normalize();
  for (unsigned i=0;i<histo.size();i++)
    cout<<histo[i]<<" ";
  cout<<endl;

}
