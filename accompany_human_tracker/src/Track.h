#ifndef Track_INCLUDED
#define Track_INCLUDED

#include <accompany_uva_msg/HumanDetections.h>
#include <accompany_uva_msg/TrackedHumans.h>

#include <KalmanFilter.h>
#include <vector>

#include <accompany_static_camera_localisation/Hull.h>

/**
 *  Represents a tracked person with kalman filter and histogram
 */
class Track
{
 public:
  Track(const geometry_msgs::PointStamped& pointStamped,double maxCovar);
  Track(const accompany_uva_msg::HumanDetection& humanDetection,double maxCovar);
  void init();

  unsigned getID();

  double match(const accompany_uva_msg::HumanDetection& humanDetection,
               const vnl_matrix<double>& obsModel,
               double stateThreshold,
               double appearanceThreshold);
  double matchState(const accompany_uva_msg::HumanDetection& humanDetection,
                    const vnl_matrix<double>& obsModel);
  double matchAppearance(const accompany_uva_msg::HumanDetection& humanDetection);

  void transition(const vnl_matrix<double>& transModel,
                  const vnl_matrix<double>& transCovariance,
                  double maxCovar);
  void observation(const accompany_uva_msg::HumanDetection& humanDetection,
                   const double appearanceUpdate,
                   const vnl_matrix<double>& obsModel,
                   const vnl_matrix<double>& obsCovariance,
                   double maxCovar);

  bool isRobot();
  void setRobot(bool value=true);

  bool isHuman();
  void setHuman(bool value=true);

  void updateRobot(const geometry_msgs::PointStamped& pointStamped,
                   const vnl_matrix<double>& obsModel,
                   const vnl_matrix<double>& obsCovariance,
                   double maxCovar);

  double distanceTo(Track &track);

  vnl_vector<double> getPosition();
  vnl_vector<double> getSpeed();
  

  void maxSpeed(double maxSpeed);

  void addUnmatchCount();

  WorldPoint toWorldPoint();

  void reduceSpeed(double reduction=0.8);

  void writeMessage(accompany_uva_msg::TrackedHuman& trackedHuman);

  friend std::ostream& operator<<(std::ostream& out,const Track& track);

  unsigned matchCount; // number of times matched by observation
  unsigned unmatchedCount; // number consecutive unmatched
  bool isRobotFlag;
  double humanProb;
  bool isHumanFlag;
  
 private:
  static unsigned nextID;
  unsigned id;
  KalmanFilter<double> kalmanFilter;
  accompany_uva_msg::Appearance appearance;
  bool appearanceValid;

  static vnl_vector<double> getObs(const accompany_uva_msg::HumanDetection& humanDetection);
  static vnl_matrix<double> getObsCovariance(const accompany_uva_msg::HumanDetection& humanDetection);
  void updateAppearance(double weight,const accompany_uva_msg::Appearance& newAppearance);
  void limitCovariance(double max);

};

#endif
