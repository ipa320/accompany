#ifndef Histogram_INCLUDED
#define Histogram_INCLUDED

#include <vector>
#include <iostream>


template <unsigned BASE,unsigned EXP>
class Power
{
private:
  std::vector< unsigned > powCache;

public:

  Power()
  {
    powCache.resize(EXP+1);
    for (unsigned e=0;e<EXP+1;e++)
      powCache[e]=pow(e);
  }

  unsigned operator()(unsigned exp)
  {
    return powCache[exp];
  }

  unsigned pow(unsigned exp)
  {
    unsigned ret=1;
    for (unsigned i=0;i<exp;i++)
      ret*=BASE;
    return ret;
  }

};

template <class TYPE,unsigned BINS,unsigned DIM> // Histogram with min and max as any type
class Histogram
{
private:
  TYPE min,max;
  unsigned count;
  std::vector<unsigned> hist;
  static Power<BINS,DIM> power;

 public:
  Histogram(TYPE min,TYPE max)
  {
    this->min=min;
    this->max=max;
    hist.resize(power(DIM));
    clear();
  }

  void clear()
  {
    count=0;
    for (unsigned i=0;i<hist.size();i++)
      hist[i]=0;
  }

  void add(const std::vector<TYPE>& data)
  {
    unsigned ind=0;
    for (unsigned i=0;i<DIM;i++)
    {
      unsigned b=BINS*(data[i]-min)/(max-min);
      ind+=b*power(i);
    }
    count++;
    hist[ind]++;
  }

  std::vector<float> normalize()
  {
    std::vector<float> h(hist.size());
    for (unsigned i=0;i<hist.size();i++)
      h[i]=hist[i]/((float)count);
    return h;
  }

};

// init static member
template <class TYPE,unsigned BINS,unsigned DIM>
Power<BINS,DIM> Histogram<TYPE,BINS,DIM>::power=Power<BINS,DIM>();


template <class TYPE,unsigned BINS,unsigned DIM,int MIN,int MAX> // Histogram with min and max as int
class HistogramInt
{
private:
  unsigned count;
  std::vector<unsigned> hist;
  static Power<BINS,DIM> power;

 public:
  HistogramInt()
  {
    hist.resize(power(DIM));
    clear();
  }

  void clear()
  {
    count=0;
    for (unsigned i=0;i<hist.size();i++)
      hist[i]=0;
  }

  void add(const std::vector<TYPE>& data)
  {
    unsigned ind=0;
    for (unsigned i=0;i<DIM;i++)
    {
      unsigned b=BINS*(data[i]-MIN)/(MAX-MIN);
      ind+=b*power(i);
    }
    count++;
    hist[ind]++;
  }

  std::vector<float> normalize()
  {
    std::vector<float> h(hist.size());
    for (unsigned i=0;i<hist.size();i++)
      h[i]=hist[i]/((float)count);
    return h;
  }

};

// init static member
template <class TYPE,unsigned BINS,unsigned DIM,int MIN,int MAX>
Power<BINS,DIM> HistogramInt<TYPE,BINS,DIM,MIN,MAX>::power=Power<BINS,DIM>();


#endif
