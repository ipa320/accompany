#ifndef BACKGROUND_HH
#define BACKGROUND_HH

#include <list>
#include <vector>
#include <opencv/cv.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include "defines.hh"
#include <data/XmlPackable.hh>

class Background : public XmlPackable {
protected:
public:
     unsigned N;                // Max n.o. images
     unsigned d;                // Kept n.o. eigenvectors
     FLOAT n;                  // Real n.o. images
     std::list< vnl_vector<FLOAT> > imgs; // images
     std::list< vnl_vector<FLOAT> > valImgs; // validation images
     
     std::vector< vnl_vector<FLOAT> > eigenvectors;

     // void innerproduct(vnl_matrix<FLOAT> &S);
     
public:
     vnl_vector<FLOAT> sum, sumsq, mu, var; // Sum of all images, mean

     Background(unsigned numImg=0, unsigned numVec=0)
          : N(numImg), d(numVec), n(0)
          {}

     void processImage(const vnl_vector<FLOAT> &img);
     void update();
     void dropImg() { imgs.clear(); valImgs.clear(); }

     void getBackground(const vnl_vector<FLOAT> &img,
                        vnl_vector<FLOAT> &bg);
     void getWeightedSqDiff(const vnl_vector<FLOAT> &img,
                            vnl_vector<FLOAT> &diff);
     void segment(const vnl_vector<FLOAT> &img,
                  vnl_vector<FLOAT> &bg,
                  vnl_vector<FLOAT> &fg,
                  std::vector<int> &mask);
     void segment(const vnl_vector<FLOAT> &img,
                  std::vector<int> &mask);
     void getProjection(const vnl_vector<FLOAT> &img,
                        vnl_vector<FLOAT> &proj);

     void xmlPack(XmlFile &f) const;
     void xmlUnpack(XmlFile &f);
};
     
IplImage *vec2img(const vnl_vector<FLOAT> &v);
IplImage *bmp2img(const vnl_vector<FLOAT> &v);
void img2vec(const IplImage *img, vnl_vector<FLOAT> &v);
void showImg(const char *win, const vnl_vector<FLOAT> &v, bool convert=false);

void getBackground(const char *filename, Background &bg);
void getBackground(const char *filename, std::vector<Background> &bg, unsigned C=3,unsigned smooth=0);


#endif  // BACKGROUND_HH
