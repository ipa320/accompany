#include <Track.h>

#include <math.h>
#include <vnl/algo/vnl_determinant.h>
using namespace std;

#define DIMENSION 2
#define KL_MIN 0.00000001


unsigned Track::nextID=0;

vnl_matrix<double> toMatrix(vnl_vector<double> v)
{
  vnl_matrix<double> m(v.size(),1);
  for (unsigned i=0;i<v.size();i++)
    m[i][0]=v[i];
  return m;
}

vnl_vector<double> addSpeed(vnl_vector<double> v)
{
  vnl_vector<double> s(v.size()+2,0);
  for (unsigned i=0;i<v.size();i++)
    s[i]=v[i];
  return s;
}

vnl_matrix<double> addSpeed(vnl_matrix<double> m)
{
  vnl_matrix<double> s(m.rows()+2,m.columns()+2);
  s.set_identity();
  for (unsigned i=0;i<m.rows();i++)
    for (unsigned j=0;j<m.columns();j++)
      s[i][j]=m[i][j];
  return s;
}

vnl_vector<double> getPoint(const geometry_msgs::PointStamped& pointStamped)
{
  vnl_vector<double> state(DIMENSION);
  state[0]=pointStamped.point.x;
  state[1]=pointStamped.point.y;
  return state;
}

vnl_matrix<double> getPointCovariance(const geometry_msgs::PointStamped& pointStamped)
{
  vnl_matrix<double> covariance(DIMENSION,DIMENSION);
  covariance.set_identity();
  return covariance;
}

/**
 * Constructor
 * @param pointStamped detection info to initialize a track
 * @param maxCovar maxium of covariance diagonal elements
 */
Track::Track(const geometry_msgs::PointStamped& pointStamped,double maxCovar)
  : kalmanFilter(addSpeed(getPoint(pointStamped)),
                 addSpeed(getPointCovariance(pointStamped))*maxCovar/2)
{
  appearanceValid=false;
  init();
  matchCount=100;
  isRobotFlag=true;
}

/**
 * Constructor
 * @param humanDetection detection info to initialize a track
 * @param maxCovar maxium of covariance diagonal elements
 */
Track::Track(const accompany_uva_msg::HumanDetection& humanDetection,double maxCovar)
  : kalmanFilter(addSpeed(getObs(humanDetection)),
                 addSpeed(getObsCovariance(humanDetection))*maxCovar/2)
{
  appearance=humanDetection.appearance;
  appearanceValid=true;
  init();
}

void Track::init()
{
  id=++nextID;
  matchCount=0;
  unmatchedCount=0;
  humanProb=0;
  isRobotFlag=false;
  isHumanFlag=false;
}

bool Track::isRobot()
{
  return isRobotFlag;
}

void Track::setRobot(bool value)
{
  isRobotFlag=value;
}

bool Track::isHuman()
{
  return isHumanFlag;
}

void Track::setHuman(bool value)
{
  isHumanFlag=value;
}

/**
 * Compute the match of this track with the detection
 * @param humanDetection human detection
 * @param obsModel observation model that maps the kalman state to the kalman observation space
 * @param stateThreshold a threshold on the kalman state
 * @param appearanceThreshold a threshold on the appearance
 * @return the match value
 */
double Track::match(const accompany_uva_msg::HumanDetection& humanDetection,
                    const vnl_matrix<double>& obsModel,
                    double stateThreshold,
                    double appearanceThreshold)
{
  double state=matchState(humanDetection,obsModel);
  
  cout<<id<<" state-match-score: "<<state;
  if (state<stateThreshold)
  {
    cout<<" < stateThreshold="<<stateThreshold<<endl;
    return -numeric_limits<double>::max();
  }
  else
    cout<<endl;

  double appearance=appearanceThreshold/2;// default
  if (appearanceValid)
  {
    appearance=matchAppearance(humanDetection);
    cout<<id<<" appearance-match-score: "<<appearance;
    if (appearance<appearanceThreshold)
    {
      cout<<" < appearanceThreshold="<<appearanceThreshold<<endl;
      return -numeric_limits<double>::max();
    }
    else
      cout<<endl;
  }
  
  return state*appearance;
}

/**
 * Return unique ID associated with this track
 * @return the ID
 */
unsigned Track::getID()
{
  return id;
}

/**
 * Compute the match of this track with the detection based on the kalman state
 * @param humanDetection human detection information
 * @param obsModel observation model that maps the kalman state to the kalman observation space
 * @return the match value
 */
double Track::matchState(const accompany_uva_msg::HumanDetection& humanDetection,
                         const vnl_matrix<double>& obsModel)
{
  vnl_vector<double> state=obsModel*kalmanFilter.getState();
  vnl_matrix<double> covar=obsModel*kalmanFilter.getCovariance()*obsModel.transpose();
  vnl_vector<double> obs=getObs(humanDetection);
  vnl_matrix<double> diff=toMatrix(obs-state);
  double det=vnl_determinant(covar);
  vnl_matrix<double> inv=vnl_matrix_inverse<double>(covar);
  vnl_matrix<double> expo=diff.transpose()*inv*diff;
  double match=/*1/(sqrt(pow(2*M_PI,2)*det))**/exp(-(double)(expo[0][0])/2.0);
  
  /*
  cout<<endl;
  cout<<"---------- matchState"<<endl;
  cout<<"state:"<<state<<endl;
  cout<<"covar:"<<endl<<covar;
  cout<<"obs:"<<obs<<endl;
  cout<<"diff:"<<endl<<diff;
  cout<<"det:"<<det<<endl;
  cout<<"inv:"<<endl<<inv;
  cout<<"expo:"<<endl<<expo;
  cout<<"match: "<<match<<endl;
  */

  return match;
}

/**
 * Compute the match of this track with the detection based on the appearance
 * @param humanDetection human detection
 * @return the match value
 */
double Track::matchAppearance(const accompany_uva_msg::HumanDetection& humanDetection)
{
  /* // 
  unsigned dim=appearance.histogram.size();
  vnl_matrix<double> diff(dim,1);
  double var=0.2;
  for (unsigned i=0;i<dim;i++)
    diff[i][0]=appearance.histogram[i]-humanDetection.appearance.histogram[i];
  vnl_matrix<double> exp=diff.transpose()*diff/var;
  double match=-exp[0][0];
  return match;
  */
  double intersect=0;
  for (unsigned i=0;i<appearance.histogram.size();i++)
    intersect+=(appearance.histogram[i]<humanDetection.appearance.histogram[i])?
      appearance.histogram[i]:
      humanDetection.appearance.histogram[i]; // min
  return intersect;
}

/**
 * Kalman prediction step, moves track forward based on known speed. The appearance is unchanged.
 * @param transModel kalman transition model 
 * @param transCovariance kalman transition covariance
 */
void Track::transition(const vnl_matrix<double>& transModel,
                       const vnl_matrix<double>& transCovariance,
                       double maxCovar)
{
  kalmanFilter.transition(transModel,
                          transCovariance);
  limitCovariance(maxCovar);
}

/**
 * Updates the track with a detection. This includes a Kalman observation update step and updating the appearance
 * @param humanDetection human detection information
 * @param obsModel kalman observation model
 * @param obsCovariance kalman observation covariance
 */
void Track::observation(const accompany_uva_msg::HumanDetection& humanDetection,
                        const double appearanceUpdate,
                        const vnl_matrix<double>& obsModel,
                        const vnl_matrix<double>& obsCovariance,
                        double maxCovar)
{
  vnl_vector<double> obs=getObs(humanDetection);
  kalmanFilter.observation(obs,
                           obsModel,
                           obsCovariance);
  if (appearanceValid)
  {
    updateAppearance(appearanceUpdate,humanDetection.appearance);
  }
  else
  {
    appearance=humanDetection.appearance;
    appearanceValid=true;
  }
  matchCount++;
  unmatchedCount=0;
  limitCovariance(maxCovar);
}

/**
 * Updates the track with a robot tf point
 * @param humanDetection human detection information
 * @param obsModel kalman observation model
 * @param obsCovariance kalman observation covariance
 */
void Track::updateRobot(const geometry_msgs::PointStamped& pointStamped,
                        const vnl_matrix<double>& obsModel,
                        const vnl_matrix<double>& obsCovariance,
                        double maxCovar)
{
  vnl_vector<double> obs=getPoint(pointStamped);
  kalmanFilter.observation(obs,
                           obsModel,
                           obsCovariance);
  matchCount++;
  unmatchedCount=0;
  limitCovariance(maxCovar);
}

/**
 * Returns distance from this track to other track
 * @param track to which to compute distance
 * @return distance
 */
double Track::distanceTo(Track &track)
{
  cout<<"v1: "<<this->getPosition()<<" v2:"<<track.getPosition()<<" distance:"<<sqrt(vnl_vector_ssd(this->getPosition(),track.getPosition()))<<endl;
  return sqrt(vnl_vector_ssd(this->getPosition(),track.getPosition()));
}

/**
 * Reduces the speed to maxSpeed
 */
void Track::maxSpeed(double maxSpeed)
{
  double dx=kalmanFilter.getState()[2];
  double dy=kalmanFilter.getState()[3];
  double speed=sqrt(dx*dx+dy*dy);
  if (speed>maxSpeed)
  {
    double factor=maxSpeed/speed;
    kalmanFilter.getState()[2]*=factor;
    kalmanFilter.getState()[3]*=factor;
  }
}

/**
 * Increase consecutive unmatch count. This count it set to '0' in function observation() when a match is found.
 */
void Track::addUnmatchCount()
{
  unmatchedCount++;
}

/**
 * Convert position to WorldPoint
 * @returns position in WorldPoint format
 */
WorldPoint Track::toWorldPoint()
{
  WorldPoint wp(kalmanFilter.getState()[0],
                kalmanFilter.getState()[1],
                0);
  wp*=1000.0; // from meters to millimeters
  return wp;
}

void Track::reduceSpeed(double reduction)
{
  kalmanFilter.getState()[2]*=reduction;
  kalmanFilter.getState()[3]*=reduction;
}

/**
 * Updates the appearance of the track
 * @param weight the weight of the new appearance relative to the old
 * @param newAppearance the new appearance
 */
void Track::updateAppearance(double weight,const accompany_uva_msg::Appearance& newAppearance)
{
  double weight1=(1-weight);
  double weight2=weight;  
  for (unsigned i=0;i<appearance.histogram.size();i++)
  {
    appearance.histogram[i]=appearance.histogram[i]*weight1+newAppearance.histogram[i]*weight2;
  }
  appearance.sumPixelWeights=appearance.sumPixelWeights*weight1+newAppearance.sumPixelWeights*weight2;
  appearance.sumTemplatePixelSize=appearance.sumTemplatePixelSize*weight1+newAppearance.sumTemplatePixelSize*weight2;
}

vnl_vector<double> Track::getPosition()
{
  vnl_vector<double> v(3,0);
  v[0]=kalmanFilter.getState()[0];
  v[1]=kalmanFilter.getState()[1];
  v[2]=0;
  return v;
}

vnl_vector<double> Track::getSpeed()
{
  vnl_vector<double> v(3,0);
  v[0]=kalmanFilter.getState()[2];
  v[1]=kalmanFilter.getState()[3];
  v[2]=0;
  return v;
}

/**
 * Convert the track information to a TrackedHuman message
 * @param trackedHuman the message to write to
 */
void Track::writeMessage(accompany_uva_msg::TrackedHuman& trackedHuman)
{
  trackedHuman.location.point.x=kalmanFilter.getState()[0];
  trackedHuman.location.point.y=kalmanFilter.getState()[1];
  trackedHuman.location.point.z=0;
  trackedHuman.speed.vector.x=kalmanFilter.getState()[2];
  trackedHuman.speed.vector.y=kalmanFilter.getState()[3];
  trackedHuman.speed.vector.z=0;
  trackedHuman.id=id;
  if (this->isRobot())
    trackedHuman.identity="robot";
  else
    trackedHuman.identity="";
  if (this->isHuman())
    trackedHuman.specialFlag=1;
  else
    trackedHuman.specialFlag=0;
}

/**
 * Get the kalman observation from a detection
 * @param humanDetection human detection information
 */
vnl_vector<double> Track::getObs(const accompany_uva_msg::HumanDetection& humanDetection)
{
  vnl_vector<double> state(DIMENSION);
  state[0]=humanDetection.location.point.x;
  state[1]=humanDetection.location.point.y;
  return state;
}

/**
 * Get the kalman observation covariance based on the detection dimensionality
 * @param humanDetection human detection information
 */
vnl_matrix<double> Track::getObsCovariance(const accompany_uva_msg::HumanDetection& humanDetection)
{
  vnl_matrix<double> covariance(DIMENSION,DIMENSION);
  covariance.set_identity();
  return covariance;
}

/**
 * ostreams a track
 */
std::ostream& operator<<(std::ostream& out,const Track& track)
{
  out<<"Track: "<<track.id<<endl;
  out<<track.kalmanFilter;
  if (track.appearanceValid)
  {
    out<<"appearance:"<<endl;
    out<<"  sumTemplatePixelSize:"<<track.appearance.sumTemplatePixelSize<<endl;
    out<<"  sumPixelWeights:"<<track.appearance.sumPixelWeights<<endl;
    out<<"  matchCount:"<<track.matchCount<<endl;
    out<<"  unmatchedCount:"<<track.unmatchedCount<<endl;
    out<<"  isRobotFlag:"<<track.isRobotFlag<<endl;
    out<<"  humanProb:"<<track.humanProb<<endl;
    out<<"  isHumanFlag:"<<track.isHumanFlag<<endl;
    out<<"  ";
    for (unsigned i=0;i<track.appearance.histogram.size();i++)
      out<<track.appearance.histogram[i]<<" ";
    out<<endl;
  }
  return out;
}

/**
 * Limit the covariance to max
 * @param max maximum value of elements in covariance matrix
 */
void Track::limitCovariance(double max)
{
  // limit covariance
  vnl_matrix<double>& covar=kalmanFilter.getCovariance();
  for (unsigned i=0;i<covar.rows();i++)
    for (unsigned j=0;j<covar.cols();j++)
      if (covar(i,j)>max)
        covar(i,j)=max;
}
