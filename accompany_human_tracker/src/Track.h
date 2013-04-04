#ifndef Track_INCLUDED
#define Track_INCLUDED

#include <accompany_uva_msg/HumanDetections.h>
#include <accompany_uva_msg/TrackedHumans.h>

#include <KalmanFilter.h>
#include <vector>

class Track
{
 public:
  
  Track(const accompany_uva_msg::HumanDetection& humanDetection);
  
  static vnl_vector<double> getState(const accompany_uva_msg::HumanDetection& humanDetection);
  static vnl_matrix<double> getCovariance(const accompany_uva_msg::HumanDetection& humanDetection);
  void setSpeed(vnl_vector<double>& detection);

  double match(const accompany_uva_msg::HumanDetection& humanDetection);
  double matchState(const accompany_uva_msg::HumanDetection& humanDetection);
  double matchAppearance(const accompany_uva_msg::HumanDetection& humanDetection);
  void update(const accompany_uva_msg::HumanDetection& humanDetection);

  friend std::ostream& operator<<(std::ostream& out,const Track& track);

 private:
  KalmanFilter<double> kalmanFilter;
  accompany_uva_msg::Appearance appearance;

  
}; 

#endif
