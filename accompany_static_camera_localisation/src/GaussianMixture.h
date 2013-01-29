#ifndef GaussianMixture_INCLUDED
#define GaussianMixture_INCLUDED

#include <vector>
#include <iostream>
#include <Gaussian.h>

/**
 *  A mixture of multivariate Gaussians with diagonal covariance matrices.
 */
template <class T,unsigned int dimension,unsigned int maxNrGaussian>
class GaussianMixture
{
 public:

  /**
   * Constructor
   * @param dimension dimension of the gaussians
   * @param maxNrGaussian maximum number of gaussians
   */
  GaussianMixture()
  {
    gaussians=new Gaussian<T,dimension>*[maxNrGaussian];
    for (unsigned int i=0;i<maxNrGaussian;i++)
      gaussians[i]=NULL;
    currentNrGaussian=0;
  }

  /**
   * Destructor
   */
  ~GaussianMixture()
  {
    for (unsigned int i=0;i<maxNrGaussian;i++)
      if (gaussians[i]!=NULL)
        delete gaussians[i];
    delete[] gaussians;
  }

  /**
   * Get current number of gaussians
   * @return current number of gaussians
   */
  unsigned int getNrGaussians()
  {
    return currentNrGaussian;
  }

  /**
   * Get gaussian from the mixture by index
   * @param gaussianID index of gaussian 
   * @return gaussian
   */
  Gaussian<T,dimension> *getGaussian(unsigned int gaussianIndex)
  {
    return gaussians[gaussianIndex];
  }

  /**
   * Initialize gaussian mixture
   * @param gaussians Gaussians of the mixture
   * @param nrGaussian number of Gaussians
   */
  void init(Gaussian<T,dimension> *gaussians,int nrGaussian)
  {
    currentNrGaussian=nrGaussian;
    for (int i=0;i<currentNrGaussian;i++)
    {
      if (this->gaussians[i]!=NULL)
        delete this->gaussians[i];
      this->gaussians[i]=new Gaussian<T,dimension>(gaussians[i]);
    }
    normalize();
  }

  /**
   * Probability density of data point. To be used when the data point will be used to update the mixture.
   * The sum of the probability to each gaussian multiplied by its weight.
   * @param data the data point
   * @param squareDist is used for intermediate data storage, should have at least the same dimension as point
   * @param minWeight the minimal weight a gaussian to be included in the sum, set to '0' to include all gaussians
   * @param squareMahanobisMatch the maximum square Mahanobis distance to a gaussian for it to selected for update
   * @param updateGaussianID provides the id of the gaussian to be updated (heighest weight and withing squareMahanobisMatch distance)
   * @return the probability density of data point
   */
  T probability(T *data,T *squareDist,const T &minWeight,const T &squareMahanobisMatch,int &updateGaussianID)
  {
    T prob=0;
    T bestWeight=0;
    updateGaussianID=-1;
    for (unsigned int i=0;i<currentNrGaussian;i++)
    {
      T squareMahanobis=gaussians[i]->squareMahanobisDistance(gaussians[i]->squareDistance(data,squareDist));
      //cout<<"squareMahanobis:"<<squareMahanobis<<endl;
      if (squareMahanobis<squareMahanobisMatch)
      {
        if (gaussians[i]->weight>bestWeight)
        {
          bestWeight=gaussians[i]->weight;
          updateGaussianID=i;
        }
      }
      if (gaussians[i]->weight>minWeight)
        prob+=gaussians[i]->weight*gaussians[i]->probability(squareMahanobis);
    }
    return prob;
  }

  /**
   * Updates the gaussian mixture
   * @param data the data point
   * @param decay the decay facotor, the weight of the new point relative to all earlier points
   * @param initVar inital variance used when creating a new gaussian
   * @param updateGaussianID the id of the gaussian to be updated
   */
  void update(T *data,const T &initVar,const T &decay,const T &weightReduction,const int &updateGaussianID)
  {
    if (updateGaussianID<0) // new gaussian
    {
      if (currentNrGaussian<maxNrGaussian) // create new
      {
        if (gaussians[currentNrGaussian]==NULL)
          gaussians[currentNrGaussian]=new Gaussian<T,dimension>;
        currentNrGaussian+=1;
      }
      gaussians[currentNrGaussian-1]->init(data,initVar,decay); // init
    }
    else // update gaussian
    {
      for (unsigned int i=0;i<currentNrGaussian;i++)
        gaussians[i]->update(data,decay,weightReduction,(int)(i==updateGaussianID));
    }
    sort(updateGaussianID);
    normalize();
  }

  /**
   * Sample a data point from the gaussian mixture distribution
   * @param data the resulting sample point
   */
  void sample(T *data)
  {
    T s=((T)rand())/RAND_MAX;
    T w=0;
    for (int i=0;i<currentNrGaussian;i++)
    {
      w+=gaussians[i]->weight;
      if (s<w)
      {
        gaussians[i]->sample(data);
        break;
      }
    }
  }

  /**
   * Streams a GaussianMixture. Because of this function we print using "cout<<gaussianMixture<<endl;".
   * @param out the stream
   * @param gaussianMixture the GaussianMixture
   * @return the stream
   */
  friend std::ostream &operator<<(std::ostream &out,const GaussianMixture &gaussianMixture)
  {
    out<<"GaussianMixture"<<std::endl;
    for (unsigned int i=0;i<gaussianMixture.currentNrGaussian;i++)
      out<<"  "<<*(gaussianMixture.gaussians[i])<<std::endl;
    return out;
  }

  // -------------- private --------------
 private:

  void sort(const int &updateGaussianID)
  {
    // sort 
    int i=updateGaussianID;
    while (i>0)
    {
      if (gaussians[i]->weight>gaussians[i-1]->weight)
      {
        Gaussian<T,dimension> *temp=gaussians[i-1];
        gaussians[i-1]=gaussians[i];
        gaussians[i]=temp;
      }
      else
        break;
      i--;
    }
    
    // remove gaussians with negative weight 
    unsigned int j=currentNrGaussian-1;
    while (gaussians[j]->weight<0)
    {
      currentNrGaussian--;
      if (j--==0) break;
    }
  }

  void normalize()
  {
    T sum=0;
    for (unsigned int i=0;i<currentNrGaussian;i++)
      sum+=gaussians[i]->weight;
    for (unsigned int i=0;i<currentNrGaussian;i++)
      gaussians[i]->weight/=sum;
  }

  unsigned int currentNrGaussian;
  Gaussian<T,dimension> **gaussians;

};

#endif
