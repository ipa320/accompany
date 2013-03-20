#ifndef Histogram_INCLUDED
#define Histogram_INCLUDED

#include <vector>
#include <iostream>

/**
 * Precomputes al the exponents of BASE from BASE^0 through BASE^EXP.
 */
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

  /**
   * Take BASE to exp exponent
   * @param exp the exponent
   * @returns BASE^exp
   */
  unsigned operator()(unsigned exp)
  {
    return powCache[exp];
  }

 private:
  unsigned pow(unsigned exp)
  {
    unsigned ret=1;
    for (unsigned i=0;i<exp;i++)
      ret*=BASE;
    return ret;
  }

};

/**
 * Builds a histrogram of multidimensional incomming data
 * @param TYPE_DATA type of data elements
 * @param TYPE_WEIGHT type of weights
 * @param BINS number of bins in each dimension
 * @param DIM dimension of data
 */
template <class TYPE_DATA,class TYPE_WEIGHT,unsigned BINS,unsigned DIM> // Histogram with min and max as any type
class Histogram
{
protected:
  TYPE_DATA min,max;
  TYPE_WEIGHT count;
  std::vector<TYPE_WEIGHT> hist;
  static Power<BINS,DIM> power;

 public:
  
  /**
   * Constructor
   * @param min minimal value in each dimension
   * @param max value higher than the maximum value in each dimension
   */
  Histogram(TYPE_DATA min,TYPE_DATA max)
  {
    this->min=min;
    this->max=max;
    hist.resize(power(DIM));
    clear();
  }

  /**
   * Copy Constructor
   */
  Histogram(const Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>& histogram)
  {
    this->min=histogram.min;
    this->max=histogram.max;
    this->count=histogram.count;
    this->hist=histogram.hist;
  }

  /**
   * Forget previously received data
   */
  void clear()
  {
    count=0;
    for (unsigned i=0;i<hist.size();i++)
      hist[i]=0;
  }

  /**
   * Add data to the histogram
   * @param data multidimensional data point to add
   * @param weight the weight of the data point
   */
  void add(const std::vector<TYPE_DATA>& data,const TYPE_WEIGHT weight)
  {
    unsigned ind=0;
    for (unsigned i=0;i<DIM;i++)
      ind+=bin(data[i])*power(i);
    count+=weight;
    hist[ind]+=weight;
  }

  /**
   * Add data to the histogram
   * @param data multidimensional data point to add
   * @param weight the weight of the data point
   */
  void add(TYPE_DATA *data,const TYPE_WEIGHT weight)
  {
    unsigned ind=0;
    for (unsigned i=0;i<DIM;i++)
    {
      ind+=bin(data[i])*power(i);
    }
    count+=weight;
    hist[ind]+=weight;
  }

  /**
   * Add histogram to this histogram
   * @param histogram the histogram to add
   */
  void operator+=(const Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>& histogram)
  {
    for (unsigned i=0;i<hist.size();i++)
      hist[i]+=histogram.hist[i];
    count+=histogram.count;
  }

  /**
   * Add histogram to this histogram
   * @param histogram the histogram to add
   */
  Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM> operator+(const Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>& histogram)
  {
    Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM> ret(*this);
    ret+=histogram;
    return ret;
  }

  /**
   * Normalize the received data and return the histogram
   * @returns normalized histogram data
   */
  std::vector<TYPE_WEIGHT> normalize()
  {
    std::vector<TYPE_WEIGHT> h(hist.size());
    for (unsigned i=0;i<hist.size();i++)
      h[i]=hist[i]/(count);
    return h;
  }

  /**
   * Allows histogram to be io streamed
   */
  friend std::ostream &operator<<(std::ostream &out,const Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>& histogram)
  {
    out<<histogram.count<<": ";
    for (unsigned i=0;i<histogram.hist.size();i++)
      out<<histogram.hist[i]<<" ";
    return out;
  }

 private:
  
  virtual unsigned bin(TYPE_DATA d)
  {
    return (BINS*(d-min))/(max-min);
  }

};

/**
 * Builds a histrogram of multidimensional incomming data. It differs from class Histogram in that the min and max 
 * are now template variables so the compiler can optimize computation in the bin() method
 * @param TYPE_DATA type of data elements
 * @param TYPE_WEIGHT type of weights
 * @param BINS number of bins in each dimension
 * @param DIM dimension of data
 * @param MIN minimal value in each dimension
 * @param MAX value higher than the maximum value in each dimension (max+1)
 */
template <class TYPE_DATA,class TYPE_WEIGHT,unsigned BINS,unsigned DIM,int MIN,int MAX> // Histogram with min and max as int
class HistogramInt : public Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>
{

 public:

  /**
   * Constructor
   */
  HistogramInt()
    : Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>(MIN,MAX)
  {
  }

  /**
   * Copy Constructor
   */
  HistogramInt(const HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX>& histogram)
    : Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>(MIN,MAX)
  {
    this->min=histogram.min;
    this->max=histogram.max;
    this->count=histogram.count;
    this->hist=histogram.hist;
  }

  /**
   * Add histogram to this histogram
   * @param histogram the histogram to add
   */
  void operator+=(const HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX>& histogram)
  {
    *((Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>*)this)+=(const Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>)histogram;
  }

  /**
   * Add histogram to this histogram
   * @param histogram the histogram to add
   */
  HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX> operator+(const HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX>& histogram)
  {
    HistogramInt<TYPE_DATA,TYPE_WEIGHT,BINS,DIM,MIN,MAX> ret(*this);
    ret+=histogram;
    return ret;
  }

 private:

  unsigned bin(TYPE_DATA d)
  {
    return (BINS*(d-MIN))/(MAX-MIN);
  }

};

// init static member
template <class TYPE_DATA,class TYPE_WEIGHT,unsigned BINS,unsigned DIM>
Power<BINS,DIM> Histogram<TYPE_DATA,TYPE_WEIGHT,BINS,DIM>::power=Power<BINS,DIM>();

#endif
