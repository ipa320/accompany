#ifndef DataAssociation_INCLUDED
#define DataAssociation_INCLUDED

#include <vector>
#include <iostream>

/**
 * Associate detections with tracks
 */
class DataAssociation
{
 public:
  DataAssociation();
  void clear(unsigned s1,unsigned s2);

  void set(unsigned d1,unsigned d2,double association);  
  std::vector<int> associate(double threshold,int order=1);
  
  friend std::ostream& operator<<(std::ostream& out,const DataAssociation& dataAssociation);

 private:
  
  std::vector<std::vector<double> > associations;
  unsigned size1,size2;
  std::vector<int> assign1,assign2;
  
  std::pair<int,int> getMax(double threshold,int order);

};

#endif
