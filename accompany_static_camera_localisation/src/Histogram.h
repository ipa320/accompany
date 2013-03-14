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

template <class TYPE_DATA,class TYPE_WEIGHT,unsigned BINS,unsigned DIM> // Histogram with min and max as any type
class Histogram
{
protected:
  TYPE_DATA min,max;
  TYPE_WEIGHT count;
  std::vector<TYPE_WEIGHT> hist;
  static Power<BINS,DIM> power;

 public:
  Histogram(TYPE_DATA min,TYPE_DATA max)
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

  virtual unsigned bin(TYPE_DATA d)
  {
    return BINS*(d-min)/(max-min);
  }

  void add(const std::vector<TYPE_DATA>& data,const TYPE_WEIGHT weight)
  {
    unsigned ind=0;
    for (unsigned i=0;i<DIM;i++)
    {
      ind+=bin(data[i])*power(i);
    }
    count+=weight;
    hist[ind]+=weight;
  }

  std::vector<TYPE_WEIGHT> normalize()
  {
    std::vector<TYPE_WEIGHT> h(hist.size());
    for (unsigned i=0;i<hist.size();i++)
      h[i]=hist[i]/(count);
    return h;
  }

};

template <class TYPE_DATA,class TYPE_WEIGHT,unsigned BINS,unsigned DIM,int MIN,int MAX> // Histogram with min and max as int
class HistogramInt : public Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>
{

 public:

  HistogramInt()
    : Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>(MIN,MAX)
  {
  }

  unsigned bin(TYPE_DATA d)
  {
    return BINS*(d-MIN)/(MAX-MIN);
  }

};

// init static member
template <class TYPE_DATA,class TYPE_WEIGHT,unsigned BINS,unsigned DIM>
Power<BINS,DIM> Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>::power=Power<BINS,DIM>();

#endif
