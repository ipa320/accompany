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

/**
 * Constructor
 * @param humanDetection detection info to initialize a track
 */
Track::Track(const accompany_uva_msg::HumanDetection& humanDetection)
  : kalmanFilter(addSpeed(getObs(humanDetection)),
                 addSpeed(getObsCovariance(humanDetection)))
{
  appearance=humanDetection.appearance;
  id=++nextID;
  matchCount=0;
  unmatchedCount=0;
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
  if (state<stateThreshold)
  {
    cout<<"state="<<state<<" < stateThreshold="<<stateThreshold<<endl;
    return -numeric_limits<double>::max();
  }
  double appearance=matchAppearance(humanDetection);
  if (appearance<appearanceThreshold)
  {
    cout<<"appearance="<<appearance<<" < appearanceThreshold="<<appearanceThreshold<<endl;
    return -numeric_limits<double>::max();
  }
  return state+appearance;
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
  /*
  cout<<endl;
  cout<<"state:"<<state<<endl;
  cout<<"covar:"<<endl<<covar;
  cout<<"obs:"<<obs<<endl;
  cout<<"diff:"<<endl<<diff;
  */
  double det=vnl_determinant(covar);
  vnl_matrix<double> inv=vnl_matrix_inverse<double>(covar);
  //cout<<"inv:"<<endl<<inv;
  vnl_matrix<double> exp=diff.transpose()*inv*diff;
  //cout<<"exp:"<<endl<<exp;
  return -(DIMENSION*log(2*M_PI) + log(det) + exp[0][0])/2; // multivariate gaussian distribution
}

/**
 * Compute the match of this track with the detection based on the appearance
 * @param humanDetection human detection
 * @param obsModel observation model that maps the kalman state to the kalman observation space
 * @return the match value
 */
double Track::matchAppearance(const accompany_uva_msg::HumanDetection& humanDetection)
{
  unsigned dim=appearance.histogram.size();
  vnl_matrix<double> diff(dim,1);
  double var=0.02;
  for (unsigned i=0;i<dim;i++)
    diff[i][0]=appearance.histogram[i]-humanDetection.appearance.histogram[i];
  vnl_matrix<double> exp=diff.transpose()*diff/var;
  return -(DIMENSION*(log(2*M_PI) + log(var)) + exp[0][0])/2; // multivariate gaussian distribution
}

/**
 * Kalman prediction step, moves track forward based on known speed. The appearance is unchanged.
 * @param transModel kalman transition model 
 * @param transCovariance kalman transition covariance
 */
void Track::transition(const vnl_matrix<double>& transModel,
                       const vnl_matrix<double>& transCovariance)
{
  kalmanFilter.transition(transModel,
                          transCovariance);
}

/**
 * Updates the track with a detection. This includes a Kalman observation update step and updating the appearance
 * @param humanDetection human detection information
 * @param obsModel kalman observation model
 * @param obsCovariance kalman observation covariance
 */
void Track::observation(const accompany_uva_msg::HumanDetection& humanDetection,
                        const vnl_matrix<double>& obsModel,
                        const vnl_matrix<double>& obsCovariance)
{
  vnl_vector<double> obs=getObs(humanDetection);
  kalmanFilter.observation(obs,
                           obsModel,
                           obsCovariance);
  updateAppearance(0.1,humanDetection.appearance);
  matchCount++;
  unmatchedCount=0;
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
  weight=1;
  //cout<<*this<<endl;
  double weight1=appearance.sumPixelWeights*(1-weight);
  double weight2=newAppearance.sumPixelWeights*weight;
  double sumPixelWeights=weight1+weight2;
  weight1/=sumPixelWeights;
  weight2/=sumPixelWeights;
  
  for (unsigned i=0;i<appearance.histogram.size();i++)
  {
    appearance.histogram[i]=
      appearance.histogram[i]*weight1+
      newAppearance.histogram[i]*weight2;
  }
  appearance.sumPixelWeights=
    appearance.sumPixelWeights*(1-weight)+
    newAppearance.sumPixelWeights*weight;
  appearance.sumTemplatePixelSize=
    appearance.sumTemplatePixelSize*(1-weight)+
    newAppearance.sumTemplatePixelSize*weight;
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
  out<<"Track:"<<endl;
  out<<track.kalmanFilter;
  out<<"appearance:"<<endl;
  out<<"  sumTemplatePixelSize:"<<track.appearance.sumTemplatePixelSize<<endl;
  out<<"  sumPixelWeights:"<<track.appearance.sumPixelWeights<<endl;
  out<<"  matchCount:"<<track.matchCount<<endl;
  out<<"  unmatchedCount:"<<track.unmatchedCount<<endl;
  out<<"  ";
  for (unsigned i=0;i<track.appearance.histogram.size();i++)
    out<<track.appearance.histogram[i]<<" ";
  out<<endl;
  return out;
}

