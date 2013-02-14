#ifndef DataAssociation_H
#define DataAssociation_H

#include <vector>
#include <set>
#include <limits>

template < class D1,
  class D1Traits,
  class D2,
  class D2Traits,
  class DistFunc>
class DataAssociation
{
 public:
  DataAssociation()
  {
  }

  void buildDistanceMatrix(D1 d1,D2 d2)
  {
    size1=D1Traits::getSize(d1);
    size2=D2Traits::getSize(d2);
    unsigned int size=size1*size2;
    if (distanceMatrix.size()<size)
      distanceMatrix.resize(size);

    for (unsigned int i1=0;i1<size1;i1++) // build matrix
    {
      for (unsigned int i2=0;i2<size2;i2++)
      {
        distanceMatrix[i1*size2+i2]=DistFunc::squareDistance(D1Traits::getElement(d1,i1),
                                                             D2Traits::getElement(d2,i2));
      }
    }

    if (valid1.size()<size1) // all indices for d1 are made valid
      valid1.resize(size1);
    for (unsigned int i=0;i<size1;i++)
      valid1[i]=true;

    if (valid2.size()<size2) // all indices for d2 are made valid
      valid2.resize(size2);
    for (unsigned int i=0;i<size2;i++)
      valid2[i]=true;
  }

  unsigned int getSize1()
  {
    return size1;
  }

  unsigned int getSize2()
  {
    return size2;
  }

  double squareDistance(int i1,int i2)
  {
    return distanceMatrix[i1*size2+i2];
  }

  bool bestMatch(double& distance,int& index1,int& index2)
  {
    bool matchValid=false;
    distance=std::numeric_limits<double>::max();
    for (unsigned int i1=0;i1<size1;i1++)
    {
      if (valid1[i1]) // if valid (not already used in earlier match)
      {
        for (unsigned int i2=0;i2<size2;i2++)
        {
          if (valid2[i2]) // if valid (not already used in earlier match)
          {
            if (distanceMatrix[i1*size2+i2]<distance)
            {
              distance=distanceMatrix[i1*size2+i2];
              index1=i1;
              index2=i2;
              matchValid=true;
            }
          }
        }
      }
    }
    if (matchValid)
    {
      valid1[index1]=false; // make invalid (not to be reused in future matches)
      valid2[index2]=false;
    }
    return matchValid;
  }

  std::vector<std::pair<int,int> >& globalBestMatches(double &sumDistance)
  {
    globalsBestDist=std::numeric_limits<double>::max();
    unsigned int min=size1;
    bool flip12=false;
    std::set<int> remaining;
    if (size2<min)
    {
      flip12=true;
      min=size2;
      for (unsigned int i=0;i<size1;i++)
        remaining.insert(i);
    }
    else
    {
      for (unsigned int i=0;i<size2;i++)
        remaining.insert(i);
    }
    globalsBestMatches.resize(min);
    std::vector<std::pair<int,int> > indices;
    indices.resize(min);
    globalBestMatchesRecursive(0,indices,0,remaining,flip12);
    sumDistance=globalsBestDist;
    return globalsBestMatches;
  }

 private:

  double squareDistance(int i1,int i2,bool flip12)
  {
    if (flip12)
    {
      int temp=i1;
      i1=i2;
      i2=temp;
    }
    return distanceMatrix[i1*size2+i2];
  }

  void globalBestMatchesRecursive(unsigned int i,std::vector<std::pair<int,int> > indices,
                                  double sumDist,std::set<int> remaining,bool flip12)
  {
    if (i==indices.size())
    {
      globalsBestDist=sumDist;
      globalsBestMatches=indices;
    }
    else
    {
      for (std::set<int>::iterator it=remaining.begin();it!=remaining.end();it++)
      {
        int ind=*it;
        double sumDistNew=sumDist+squareDistance(i,ind,flip12);
        if (sumDistNew<globalsBestDist)
        {
          std::vector<std::pair<int,int> > indicesNew=indices;
          if (flip12)
            indicesNew[i]=std::pair<int,int>(ind,i);
          else
            indicesNew[i]=std::pair<int,int>(i,ind);
          std::set<int> remainingNew=remaining;remainingNew.erase(ind);
          globalBestMatchesRecursive(i+1,indicesNew,sumDistNew,remainingNew,flip12);
        }
      }
    }
  }

  unsigned int size1,size2;
  std::vector<double> distanceMatrix;
  std::vector<bool> valid1,valid2;
  std::vector<std::pair<int,int> > globalsBestMatches;
  double globalsBestDist;

};

#endif
