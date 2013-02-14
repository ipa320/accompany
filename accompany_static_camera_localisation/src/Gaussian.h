#ifndef Gaussian_INCLUDED
#define Gaussian_INCLUDED

#include <stdlib.h>
#include <iostream>
#include <math.h>

// output stream operator for vectors using a pair of pointer and dimension
template <class T>
std::ostream& operator<<(std::ostream& out,const std::pair<T *,int>& data)
{
  for (int i=0;i<data.second;i++)
  {
    if (i>0)
      out<<" ";
    out<<data.first[i];
  }
  return out;
}

/**
 *  A multivariate Gaussian with diagonal covariance matrix.
 */
template <class T,unsigned int dimension>
class Gaussian
{
 public:

  /**
   * Constructor
   * @param dimension dimension of the gaussian
   */
  Gaussian()
  {
    mean=new T[dimension];
    var=new T[dimension];
    weight=-1;
  }

  /**
   * Copy Constructor
   * @param gaussian gaussian to copy
   */
  Gaussian(Gaussian &gaussian)
  {
    mean=new T[dimension];
    var=new T[dimension];
    for (int i=0;i<dimension;i++)
    {
      mean[i]=gaussian.mean[i];
      var[i]=gaussian.var[i];
    }
    weight=gaussian.weight;
  }

  /**
   * Destructor
   */
  ~Gaussian()
  {
    delete[] mean;
    delete[] var;
  }

  /**
   * Initialize gaussian.
   * @param data the mean
   * @param decay the weight
   * @param initVar the variance in all dimensions
   */
  void init(T *data,const T &initVar,const T &decay)
  {
    for (unsigned int i=0;i<dimension;i++)
    {
      mean[i]=data[i];
      var[i]=initVar;
    }
    weight=decay;
  }

  /**
   * Initialize gaussian.
   * @param data the mean
   * @param decay the weight
   * @param initVar the variance
   */
  void init(T *data,T *initVar,const T &decay)
  {
    for (unsigned int i=0;i<dimension;i++)
    {
      mean[i]=data[i];
      var[i]=initVar[i];
    }
    weight=decay;
  }

  /**
   * Update gaussian.
   * @param data the new data point
   * @param decay the decay facotor, the weight of the new point relative to all earlier points
   * @param weightReduction extra weight reduction if point doesn't belongs to gaussian
   * @param ownership 1 if point belongs to gaussian, 0 if not and then weight decreases and the mean and variance are not updated
   */
  void update(T *data,const T &decay,const T &weightReduction,const int &ownership)
  {
    weight+=decay*(ownership-weight);
    if (!ownership)
      weight-=weightReduction;
    else
    {
      for (unsigned int i=0;i<dimension;i++)
      {
        T help=data[i]-mean[i];
        mean[i]+=decay*help/weight;
        var[i]+=decay*(help*help-var[i])/weight;
      }
    }
  }

  /**
   * Probability density of data point.
   * @param data the data point
   * @param squareDist is used for intermediate data storage, should have at least the same dimension as point
   * @return the probability density of data point
   */
  T probability(T *data,T *squareDist)
  {
    if (weight>0)
      return probability(squareMahanobisDistance(squareDistance(data,squareDist)));
    else
      return 0;
  }

  /* boxmuller.c           Implements the Polar form of the Box-Muller
     Transformation

     (c) Copyright 1994, Everett F. Carter Jr.
     Permission is granted by the author to use
     this software for any application provided this
     copyright notice is preserved.
  */
  /**
   * Sample from single dimensional normal distribution using Box Muller method
   * @param m the mean
   * @param s the standard deviation 
   * @return the sample
   */
  static T box_muller(const T &m,const T &s)	/* normal random variate generator */
  {				                /* mean m, standard deviation s */
    T x1, x2, w, y1;
    static T y2;
    static int use_last = 0;
    if (use_last)	/* use value from previous call */
    {
      y1 = y2;
      use_last = 0;
    }
    else
    {
      do {
        x1 = 2.0 * ((T)rand())/RAND_MAX - 1.0;
        x2 = 2.0 * ((T)rand())/RAND_MAX - 1.0;
        w = x1 * x1 + x2 * x2;
      } while ( w >= 1.0 );

      w = sqrt( (-2.0 * log( w ) ) / w );
      y1 = x1 * w;
      y2 = x2 * w;
      use_last = 1;
    }
    return( m + y1 * s );
  }

  /**
   * Sample a data point from the gaussian distribution
   * @param data the resulting sample point
   */
  void sample(T *data)
  {
    for (unsigned int i=0;i<dimension;i++)
    {   
      data[i]=box_muller(mean[i],sqrt(var[i]));
    }
  }

  /**
   * Streams a Gaussian. Because of this function we can print using "cout<<gaussian<<endl;".
   * @param out the stream
   * @param gaussian the gaussian
   * @return the stream
   */
  friend std::ostream &operator<<(std::ostream &out,const Gaussian<T,dimension> &gaussian)
  {
    out<<"weight:"<<gaussian.weight;
    out<<" mean:"<<std::pair<T *,int>(gaussian.mean,dimension);
    out<<" var:"<<std::pair<T *,int>(gaussian.var,dimension);
    return out;
  }

   /**
   * Compute square distance from data point to gaussian
   * @param data the data point
   * @param squareDist used for intermediate result, has to be same dimension as data
   * @return square distance
   */
  T *squareDistance(T *data,T *squareDist)
  {
    for (unsigned int i=0;i<dimension;i++)
    {
      T d=data[i]-mean[i];
      squareDist[i]=d*d;
    }
    return squareDist;
  }

  /**
   * Compute mahanobis distance based on square distance
   * @param squareDist square distance
   * @return mahanobis distance
   */
  T squareMahanobisDistance(T *squareDist)
  {
    T dist=0;
    for (unsigned int i=0;i<dimension;i++)
      dist+=squareDist[i]/var[i];
    return dist;
  }

  /**
   * Compute probaility based on mahanobis distance 
   * @param squareMahanobis mahanobis distance
   * @return probaility
   */
  T probability(const T &squareMahanobis)
  {
    T prob=exp(-squareMahanobis/2.0f);
    return prob/(piTerm*sqrtDeterminant(var));
  }

  T weight;
  T *mean,*var;
  
 private:

  static T sqrtDeterminant(T *var)
  {
    T determinant=var[0];
    for (unsigned int i=1;i<dimension;i++)
      determinant*=var[i];
    return sqrt(determinant);
  }

  
  static T piTerm;

};

// init static member piTerm 
template <class T,unsigned int dimension>
T Gaussian<T,dimension>::piTerm=pow((T)(2*M_PI),dimension/2.0f);

#endif
