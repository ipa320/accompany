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

Track::Track(const accompany_uva_msg::HumanDetection& humanDetection)
  : kalmanFilter(addSpeed(getState(humanDetection)),
                 addSpeed(getCovariance(humanDetection)))
{
  appearance=humanDetection.appearance;
  id=++nextID;
}

vnl_vector<double> Track::getState(const accompany_uva_msg::HumanDetection& humanDetection)
{
  vnl_vector<double> state(DIMENSION);
  state[0]=humanDetection.location.point.x;
  state[1]=humanDetection.location.point.y;
  return state;
}

vnl_matrix<double> Track::getCovariance(const accompany_uva_msg::HumanDetection& humanDetection)
{
  vnl_matrix<double> covariance(DIMENSION,DIMENSION);
  covariance.set_identity();
  return covariance;
}

/*
void Track::setSpeed(vnl_vector<double>& detection)
{
  detection[2]=detection[0]-kalmanFilter.getState()[0];
  detection[3]=detection[1]-kalmanFilter.getState()[1];
}
*/

double Track::match(const accompany_uva_msg::HumanDetection& humanDetection,
                    const vnl_matrix<double>& obsModel)
{
  double state=matchState(humanDetection,obsModel);
  double appearance=matchAppearance(humanDetection);
  cout<<"state: "<<state<<" appearance: "<<appearance<<endl;
  return state+appearance;
}

double Track::matchState(const accompany_uva_msg::HumanDetection& humanDetection,
                         const vnl_matrix<double>& obsModel)
{
  vnl_vector<double> obs=getState(humanDetection);
  // setSpeed(obs); // determine speed
  double det=vnl_determinant(kalmanFilter.getCovariance());
  vnl_matrix<double> diff=toMatrix(obs-obsModel*kalmanFilter.getState());
  //cout<<"diff:"<<diff<<endl;
  vnl_matrix<double> covar=obsModel*kalmanFilter.getCovariance()*obsModel.transpose();
  //cout<<"covar:"<<covar<<endl;
  vnl_matrix<double> inv=vnl_matrix_inverse<double>(covar);
  //cout<<"inv:"<<inv<<endl;
  vnl_matrix<double> exp=diff.transpose()*inv*diff;
  //cout<<"exp:"<<exp<<endl;
  return -(DIMENSION*log(2*M_PI) + log(det) + exp[0][0])/2; // multivariate gaussian distribution
}

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

void Track::transition(const vnl_matrix<double>& transModel,
                       const vnl_matrix<double>& transCovariance)
{
  kalmanFilter.transition(transModel,
                          transCovariance);
}

void Track::observation(const accompany_uva_msg::HumanDetection& humanDetection,
                        const vnl_matrix<double>& obsModel,
                        const vnl_matrix<double>& obsCovariance)
{
  vnl_vector<double> obs=getState(humanDetection);
  kalmanFilter.observation(obs,
                           obsModel,
                           obsCovariance);
  updateAppearance(0.1,humanDetection.appearance);
}

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

std::ostream& operator<<(std::ostream& out,const Track& track)
{
  out<<"Track:"<<endl;
  out<<track.kalmanFilter;
  out<<"appearance:"<<endl;
  out<<"  sumTemplatePixelSize:"<<track.appearance.sumTemplatePixelSize<<endl;
  out<<"  sumPixelWeights:"<<track.appearance.sumPixelWeights<<endl;
  out<<"  ";
  for (unsigned i=0;i<track.appearance.histogram.size();i++)
    out<<track.appearance.histogram[i]<<" ";
  out<<endl;
  return out;
}
