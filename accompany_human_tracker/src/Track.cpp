#include <Track.h>

#include <math.h>
#include <vnl/algo/vnl_determinant.h>
using namespace std;

#define DIMENSION 5

vnl_matrix<double> toMatrix(vnl_vector<double> v)
{
  vnl_matrix<double> m(v.size(),1);
  for (unsigned i=0;i<v.size();i++)
    m[i][0]=v[i];
  return m;
}

Track::Track(const accompany_uva_msg::HumanDetection& humanDetection)
  : kalmanFilter(getState(humanDetection),
                  getCovariance(humanDetection))
{
  appearance=humanDetection.appearance;
}

vnl_vector<double> Track::getState(const accompany_uva_msg::HumanDetection& humanDetection)
{
  vnl_vector<double> state(DIMENSION);
  state[0]=humanDetection.location.point.x;
  state[1]=humanDetection.location.point.y;
  state[2]=0; // dx
  state[3]=0; // dy
  state[4]=humanDetection.appearance.sumTemplatePixelSize/
    humanDetection.appearance.sumPixelWeights; // size
  return state;
}

vnl_matrix<double> Track::getCovariance(const accompany_uva_msg::HumanDetection& humanDetection)
{
  vnl_matrix<double> covariance(DIMENSION,DIMENSION);
  covariance.set_identity();
  return covariance*1;
}

void Track::setSpeed(vnl_vector<double>& detection)
{
  detection[2]=detection[0]-kalmanFilter.getState()[0];
  detection[3]=detection[1]-kalmanFilter.getState()[1];
}

double Track::match(const accompany_uva_msg::HumanDetection& humanDetection)
{
  return matchState(humanDetection)+matchAppearance(humanDetection);
}

double Track::matchState(const accompany_uva_msg::HumanDetection& humanDetection)
{
  vnl_vector<double> detection=getState(humanDetection);
  setSpeed(detection); // determine speed
  
  double det=vnl_determinant(kalmanFilter.getCovariance());
  vnl_matrix<double> diff=toMatrix(detection-kalmanFilter.getState());
  //cout<<"diff:"<<diff<<endl;
  vnl_matrix<double> inv=vnl_matrix_inverse<double>(kalmanFilter.getCovariance());
  //cout<<"inv:"<<inv<<endl;
  vnl_matrix<double> exp=diff.transpose()*inv*diff;
  //<<"exp:"<<exp<<endl;
  return -(DIMENSION*log(2*M_PI) + log(det) + exp[0][0])/2;
}

double Track::matchAppearance(const accompany_uva_msg::HumanDetection& humanDetection)
{
  return 0;
}

void Track::update(const accompany_uva_msg::HumanDetection& humanDetection)
{
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
