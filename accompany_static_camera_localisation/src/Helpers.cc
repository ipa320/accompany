#define HELPERS_CC 1

#include "Helpers.hh"
#include <set>
#include <list>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <err.h>
#include <opencv/highgui.h>
#include <vnl/vnl_vector.h>
#include <data/XmlFile.hh>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/time_parsers.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>
// #include <boost/date_time/gregorian/conversion.hpp>

#include <Hull.h>

using namespace std;
using namespace boost::posix_time;
using namespace boost;

unsigned width=0, height=0, halfresX=0, halfresY=0, camPosX=0, camPosY=0;
int depth=0, channels=0;
unsigned scanres=5;
unsigned w = 10, w2 = 3*w;
FLOAT stdev = 9;
FLOAT pixelVar=stdev*stdev, pv2 = 2*pixelVar, lpv2 = .5*log(M_PI * pv2);

FLOAT stdev1 = 9;
FLOAT pixelVar1=stdev1*stdev1, pv2_1 = 2*pixelVar1, lpv2_1 = .5*log(M_PI * pv2_1);
FLOAT stdev2 = 9;
FLOAT pixelVar2=stdev2*stdev2, pv2_2 = 2*pixelVar2, lpv2_2 = .5*log(M_PI * pv2_2);
FLOAT stdev3 = 9;
FLOAT pixelVar3=stdev3*stdev3, pv2_3 = 2*pixelVar3, lpv2_3 = .5*log(M_PI * pv2_3);

float camHeight, persHeight;

#define WHITE CV_RGB(255,255,255)

ostream &operator<<(ostream &os, const scanline_t &l)
{
  return os << "{" << l.line << ":" << l.start << "-" << l.end << "}";
}

void connectedComponents(vector<int> &v)
{
  int nextLabel = 1;
  vector< set<int> > links(1);

  unsigned idx = 0/*,sz = v.size()*/;
  for (unsigned i=0; i!=height; ++i)
    for (unsigned j=0; j!=width; ++j,++idx) {
      if (v[idx]) {
        set<int>
        neighbours;
        // find a neighbour
        if (idx > width && v[idx-width-1]) // NW
          neighbours.insert(v[idx-width-1]);
        if (idx >= width && v[idx-width]) // N
          neighbours.insert(v[idx-width]);
        if (idx >= width-1 && v[idx+1-width]) // NE
          neighbours.insert(v[idx+1-width]);
        if (idx > 0 && v[idx-1]) // W
          neighbours.insert(v[idx-1]);
        // if (idx+1 < sz && v[idx+1]) // E
        //      neighbours.insert(v[idx+1]);
        // if (idx+width-1 < sz && v[idx+width-1]) // SW
        //      neighbours.insert(v[idx+width-1]);
        // if (idx+width < sz && v[idx+width]) // S
        //      neighbours.insert(v[idx+width]);
        // if (idx+width+1 < sz && v[idx+width+1]) // SE
        //      neighbours.insert(v[idx+width+1]);

        if (neighbours.begin() == neighbours.end()) { // No neighbours
          // cout << "Element (" << i << "," << j << ")is fg but has no neighbours, vector.size()=" << links.size() << ", next=" << nextLabel << endl;
          set<int> s;
          s.insert(nextLabel);
          links.push_back(s);
          v[idx] = nextLabel;
          nextLabel++;
        } else {
          // v[idx] = neighbours.begin();
          v[idx] = *neighbours.begin();
          for (set<int>::iterator it=neighbours.begin(); it!=neighbours.end(); it++) {
            // cout << "Merging neighbours into list of " << *it << endl;
            links[*it].insert(neighbours.begin(), neighbours.end());
          }
        }
      }
    }

  cout << "Links are: " << endl;
  for (unsigned i=0; i!=links.size(); ++i) {
    cout << i << ": [ ";
    for (set<int>::iterator it=links[i].begin(); it!=links[i].end(); ++it)
      cout << *it << " ";
    cout << "]" << endl;
  }

  idx=0;
  for (unsigned i=0; i!=height; ++i)
    for (unsigned j=0; j!=width; ++j,++idx) {
      if (v[idx]) {
        int f = *links[v[idx]].begin();
        while (f!=*links[f].begin())
          f=*links[f].begin();
        links[v[idx]].insert(f);
        v[idx] = f;
      }
    }
}

void fillHole(vector<int> &v, unsigned position, int val)
{
  list<unsigned> pos;
  pos.push_back(position);
  unsigned sz = v.size();
  while(pos.begin()!=pos.end()) {
    unsigned p = pos.front();
    pos.pop_front();
    if (v[p])
      continue;
    v[p] = val;
    // if (p > width && !v[p-width-1]) // NW
    //      pos.push_back(p-width-1);
    if (p >= width && !v[p-width]) // N
      pos.push_back(p-width);
    // if (p >= width-1 && !v[p+1-width]) // NE
    //      pos.push_back(p+1-width);
    if (p > 0 && !v[p-1]) // W
      pos.push_back(p-1);
    if (p+1 < sz && !v[p+1]) // E
      pos.push_back(p+1);
    // if (p+width-1 < sz && !v[p+width-1]) // SW
    //      pos.push_back(v[p+width-1]);
    // if (p+width < sz && !v[p+width]) // S
    //      pos.push_back(v[p+width]);
    // if (p+width+1 < sz && !v[p+width+1]) // SE
    //      pos.push_back(v[p+width+1]);


  }
}

void connectedComponentsFH(vector<int> &v)
{
  int nextLabel = 1;
  vector< set<int> > links(1);
  list<unsigned> seeds;

  unsigned idx = 0/*,sz = v.size()*/;
  for (unsigned i=0; i!=height; ++i)
    for (unsigned j=0; j!=width; ++j,++idx) {
      if (v[idx]) {
        set<int>
        neighbours;
        // find a neighbour
        if (idx > width && v[idx-width-1]) // NW
          neighbours.insert(v[idx-width-1]);
        if (idx >= width && v[idx-width]) // N
          neighbours.insert(v[idx-width]);
        if (idx >= width-1 && v[idx+1-width])  // NE
          neighbours.insert(v[idx+1-width]);
        if (idx > 0 && v[idx-1]) // W
          neighbours.insert(v[idx-1]);
        // if (idx+1 < sz && v[idx+1]) // E
        //      neighbours.insert(v[idx+1]);
        // if (idx+width-1 < sz && v[idx+width-1]) // SW
        //      neighbours.insert(v[idx+width-1]);
        // if (idx+width < sz && v[idx+width]) // S
        //      neighbours.insert(v[idx+width]);
        // if (idx+width+1 < sz && v[idx+width+1]) // SE
        //      neighbours.insert(v[idx+width+1]);

        if (neighbours.begin() == neighbours.end()) { // No neighbours
          // cout << "Element (" << i << "," << j << ")is fg but has no neighbours, vector.size()=" << links.size() << ", next=" << nextLabel << endl;
          set<int> s;
          s.insert(nextLabel);
          links.push_back(s);
          v[idx] = nextLabel;
          nextLabel++;
        } else {
          // v[idx] = neighbours.begin();
          v[idx] = *neighbours.begin();
          for (set<int>::iterator it=neighbours.begin(); it!=neighbours.end(); it++) {
            // cout << "Merging neighbours into list of " << *it << endl;
            links[*it].insert(neighbours.begin(), neighbours.end());
          }

          if (idx > width  && v[idx-width] == 0 && v[idx+1-width] == v[idx]
                                                                       && (v[idx-1]==v[idx] || v[idx-width-1]==v[idx]))
            // seeds.push_back(idx-width);
            fillHole(v,idx-width,v[idx]);

        }
      }
    }

  // cout << "Links are: " << endl;
  // for (unsigned i=0; i!=links.size(); ++i) {
  //      cout << i << ": [ ";
  //      for (set<int>::iterator it=links[i].begin(); it!=links[i].end(); ++it)
  //           cout << *it << " ";
  //      cout << "]" << endl;
  // }

  idx=0;
  for (unsigned i=0; i!=height; ++i)
    for (unsigned j=0; j!=width; ++j,++idx) {
      if (v[idx]) {
        int f = *links[v[idx]].begin();
        while (f!=*links[f].begin())
          f=*links[f].begin();
        links[v[idx]].insert(f);
        v[idx] = f;
      }
    }
  for (list<unsigned>::iterator i=seeds.begin(); i!=seeds.end(); ++i)
    v[*i] = 2000;
}

bool operator!=(const CvPoint &p1, const CvPoint &p2)
		    {
  return p1.x!=p2.x || p1.y != p2.y;
		    }
bool operator==(const CvPoint &p1, const CvPoint &p2)
		    {
  return p1.x==p2.x && p1.y == p2.y;
		    }

float ccw(const CvPoint &p1, const CvPoint &p2, const CvPoint &p3)
{
  return float(p2.x - p1.x)*float(p3.y - p1.y) - float(p2.y - p1.y)*float(p3.x-p1.x);
}

void swap(CvPoint &p1, CvPoint &p2)
{
  CvPoint tmp = p1;
  p1 = p2;
  p2 = tmp;
}

float cot(CvPoint &p0, CvPoint &p)
{
  return float(p.x-p0.x)/float(p.y-p0.y);
}

float dist(CvPoint &p1, CvPoint &p2)
{
  float x = p1.x - p2.x, y = p1.y - p2.y;
  return x*x + y*y;
}

bool operator<(const CvPoint &p1, const CvPoint &p2)
{
  return (p1.y < p2.y) || (p1.y == p2.y && p1.x < p2.x);
}

void rmDuplicates(const vector<CvPoint> &v, vector<CvPoint> &r) {
  map<CvPoint,bool> known;
  r.clear();

  for (unsigned i=0; i!=v.size(); ++i) {
    if (known.find(v[i]) == known.end()) {
      r.push_back(v[i]);
      known[v[i]] = true;
    }
  }
}

/*
  jarvis(S)
   pointOnHull = leftmost point in S
   i = 0
   repeat
      P[i] = pointOnHull
      endpoint = S[0]         // initial endpoint for a candidate edge on the hull
      for j from 1 to |S|-1
         if (endpoint == pointOnHull) or (S[j] is on left of line from P[i] to endpoint)
            endpoint = S[j]   // found greater left turn, update endpoint
      i = i+1
      pointOnHull = endpoint
   until endpoint == P[0]      // wrapped around to first hull point
 */

void jarvis(const vector<CvPoint> &in, vector<CvPoint> &res)
{
  unsigned leftMost = 0;
  for (unsigned i=0; i<in.size(); ++i)
    if (in[i].x < in[leftMost].x)
      leftMost = i;
  CvPoint
  pointOnHull = in[leftMost];
  res.clear();
  res.push_back(pointOnHull);
  for (unsigned i=0; i!=in.size(); ++i) {
    CvPoint
    endpoint = in[0];
    for (unsigned j=1; j<in.size(); ++j)
      if ((endpoint == pointOnHull) || ccw(endpoint,in[j],pointOnHull) < 0)
        endpoint = in[j];
    pointOnHull = endpoint;
    res.push_back(pointOnHull);
    if (endpoint == res[0])
      return;
  }
}


void convexHull(/*IplImage *img, */const vector<CvPoint> &in, vector<CvPoint> &res)
{
  jarvis(in,res);
  return;

  vector<CvPoint>
  pt;
  // rmDuplicates(in,pt);
  pt = in;
  vector<CvPoint>
  hull(pt.size()+1);

  for (unsigned i=0; i!=pt.size(); ++i)
    hull[i+1] = pt[i];

  // find leftmost, bottommost point
  unsigned best = 1;
  for (unsigned i=2; i!=hull.size(); ++i)
    if (hull[i].y < hull[best].y || (hull[i].y == hull[best].y && hull[i].x < hull[best].x))
      best = i;
  swap(hull[1],hull[best]);

  for (unsigned i=2; i+1<hull.size(); ++i) {
    float
    c1 = cot(hull[1],hull[i]),
    c2 = cot(hull[1],hull[i+1]);
    if (c1 < c2) {
      swap(hull[i],hull[i+1]);
      i=1;             // ++i will be executed before next iteration!
    } else if (c1 == c2) {
      if (dist(hull[1],hull[i])<dist(hull[1],hull[i+1]))
        swap(hull[i],hull[i+1]);
    }
  }
  hull[0] = hull.back();

  unsigned M=2;
  for (unsigned i=3; i<hull.size(); ++i) {
    while (M>0&&ccw(hull[M-1],hull[M],hull[i])<=0)
      M--;
    M++;
    swap(hull[M],hull[i]);
  }
  res.resize(M);
  for (unsigned i=0; i!=res.size(); ++i)
    res[i] = hull[i+1];
  res[0] = res.back();
  // res.push_back(res[0]);
}

void plotHull(IplImage *img, const vector<CvPoint> &tpl)
{
  vector<CvPoint>
  hull;
  convexHull(tpl,hull);
  cvCircle(img, hull[0], 5, CV_RGB(255,0,0), 2);
  cvCircle(img, hull.back(), 3, CV_RGB(0,255,0), 2);
  for (unsigned i=1; i<hull.size(); ++i)
    cvLine(img, hull[i-1], hull[i], CV_RGB(50*i,50*i,50*i),2);
}


// void convexHull(const vector<CvPoint> &pt, vector<CvPoint> &res)
// {
//      vector<CvPoint> hull(pt.size()+1);
//      for (unsigned i=0; i!=pt.size(); ++i)
//           hull[i+1] = pt[i];

//      // find leftmost, bottommost point
//      unsigned best = 1;
//      for (unsigned i=2; i!=hull.size(); ++i)
//           if (hull[i].y < hull[best].y || (hull[i].y == hull[best].y && hull[i].x < hull[best].x))
//                best = i;
//      swap(hull[1],hull[best]);

//      for (unsigned i=2; i+1<hull.size(); ++i) {
//           float
//                c1 = cot(hull[1],hull[i]),
//                c2 = cot(hull[1],hull[i+1]);
//           if (c1 < c2) {
//                swap(hull[i],hull[i+1]);
//                i=1;
//           }  else if (c1 == c2) {
//                if (dist(hull[1],hull[i])<dist(hull[1],hull[i+1]))
//                     swap(hull[i],hull[i+1]);
//           }    
//      }                    
//      hull[0] = hull.back();

//      // CvFont ft;
//      // cvInitFont(&ft, CV_FONT_HERSHEY_SIMPLEX, .5, .5);

//      // for (unsigned i=0; i< hull.size(); ++i) {
//      //      char buffer[1024];
//      //      sprintf(buffer,"%d", i);
//      //      CvPoint pt = toImg(hull[i]);
//      //      cvPutText(img, buffer, cvPoint(pt.x,pt.y+10),&ft,CV_RGB(255,0,0));
//      // }

//      unsigned M=2;
//      for (unsigned i=3; i<hull.size(); ++i) {
//           while (ccw(hull[M-1],hull[M],hull[i])<0)
//                M--;
//           M++;
//           swap(hull[M],hull[i]);
//      }
//      res.resize(M);
//      for (unsigned i=0; i!=res.size(); ++i)
//           res[i] = hull[i+1];

//      // for (unsigned i=1; i<=M; ++i) {
//      //      char buffer[1024];
//      //      sprintf(buffer,"%d", i);
//      //      cvPutText(img, buffer, toImg(hull[i]),&ft,CV_RGB(0,255,0));
//      //      cvCircle(img, toImg(hull[i]),3,CV_RGB(0,255,0),2);
//      // }
// }

CvPoint project(const CvPoint &p , float z, float camHeight)
{
  CvPoint cp = toCam(p);
  float
  A = z / (camHeight-z);
  return toImg(cvPoint((A+1)*cp.x,(A+1)*cp.y));

  // float
  //      sqx = cp.x*cp.x,
  //      sqy = cp.y*cp.y,
  //      l = sqx+sqy;
  // if (l==0)
  //      return p;
  // float
  //      A = z / (camHeight-z), //camHeight/(camHeight-z) - 1.0,
  //      sqA = A*A,
  //      x2 = sqrt(sqA*sqx),
  //      y2 = sqrt(sqA*sqy);

  // if (cp.x<0)
  //      x2 = -x2;
  // if (cp.y<0)
  //      y2 = -y2;

  // return toImg(cvPoint(cp.x+x2, cp.y+y2));
}
// CvPoint project(const CvPoint &p , float z, float camHeight)
// {
//      CvPoint cp = toCam(p);
//      float
//           sqx = cp.x*cp.x,
//           sqy = cp.y*cp.y,
//           l = sqx+sqy;
//      if (l==0)
//           return p;
//      float
//           A = camHeight/(camHeight-z) - 1.0,
//           sqA = A*A,
//           x2 = sqrt(sqA*l*sqx/l),
//           y2 = sqrt(sqA*l*sqy/l);

//      if (cp.x<0)
//           x2 = -x2;
//      if (cp.y<0)
//           y2 = -y2;

//      return toImg(cvPoint(cp.x+x2, cp.y+y2));
// }

void genTemplate(const CvPoint &pt, float persHeight, float camHeight, vector<CvPoint> &points)
{
  int x = pt.x, y = pt.y;
  points.resize(8);
  points[0] = cvPoint(x-w,y-w);
  points[1] = cvPoint(x-w,y+w);
  points[2] = cvPoint(x+w,y+w);
  points[3] = cvPoint(x+w,y-w);
  points[4] = project(points[0],persHeight,camHeight);
  points[5] = project(points[1],persHeight,camHeight);
  points[6] = project(points[2],persHeight,camHeight);
  points[7] = project(points[3],persHeight,camHeight);
}

void plotTemplate(IplImage *img, const CvPoint &pt, float persHeight, float camHeight, const CvScalar &colour)
{
  vector<CvPoint> points;
  unsigned lw=1;
  genTemplate(pt,persHeight, camHeight, points);
  cvLine(img, points[0], points[1], colour,lw);
  cvLine(img, points[1], points[2], colour,lw);
  cvLine(img, points[2], points[3], colour,lw);
  cvLine(img, points[3], points[0], colour,lw);

  cvLine(img, points[4], points[5], colour,lw);
  cvLine(img, points[5], points[6], colour,lw);
  cvLine(img, points[6], points[7], colour,lw);
  cvLine(img, points[7], points[4], colour,lw);

  cvLine(img, points[0], points[4], colour,lw);
  cvLine(img, points[1], points[5], colour,lw);
  cvLine(img, points[2], points[6], colour,lw);
  cvLine(img, points[3], points[7], colour,lw);
}

void genTemplate2(const CvPoint &pt, float persHeight, float camHeight, vector<CvPoint> &points)
{
  int x = pt.x, y = pt.y, hw = w/2, mh = 4*persHeight/5;
  points.resize(12);
  points[0] = cvPoint(x-hw,y-hw);
  points[1] = cvPoint(x-hw,y+hw);
  points[2] = cvPoint(x+hw,y+hw);
  points[3] = cvPoint(x+hw,y-hw);
  points[4] = project(cvPoint(x-w,y-w),mh,camHeight);
  points[5] = project(cvPoint(x-w,y+w),mh,camHeight);
  points[6] = project(cvPoint(x+w,y+w),mh,camHeight);
  points[7] = project(cvPoint(x+w,y-w),mh,camHeight);
  points[8] = project(points[0],persHeight,camHeight);
  points[9] = project(points[1],persHeight,camHeight);
  points[10] = project(points[2],persHeight,camHeight);
  points[11] = project(points[3],persHeight,camHeight);
}

void plotTemplate2(IplImage *img, const CvPoint &pt, float persHeight, float camHeight, const CvScalar &colour)
{
  vector<CvPoint> points;
  unsigned lw=1;
  genTemplate2(pt,persHeight, camHeight, points);

  cvLine(img, points[0], points[1], colour,lw);
  cvLine(img, points[1], points[2], colour,lw);
  cvLine(img, points[2], points[3], colour,lw);
  cvLine(img, points[3], points[0], colour,lw);

  cvLine(img, points[4], points[5], colour,lw);
  cvLine(img, points[5], points[6], colour,lw);
  cvLine(img, points[6], points[7], colour,lw);
  cvLine(img, points[7], points[4], colour,lw);

  cvLine(img, points[8], points[9], colour,lw);
  cvLine(img, points[9], points[10], colour,lw);
  cvLine(img, points[10], points[11], colour,lw);
  cvLine(img, points[11], points[8], colour,lw);

  cvLine(img, points[0], points[4], colour,lw);
  cvLine(img, points[1], points[5], colour,lw);
  cvLine(img, points[2], points[6], colour,lw);
  cvLine(img, points[3], points[7], colour,lw);

  cvLine(img, points[4], points[8], colour,lw);
  cvLine(img, points[5], points[9], colour,lw);
  cvLine(img, points[6], points[10], colour,lw);
  cvLine(img, points[7], points[11], colour,lw);
}



CvPoint toCam(CvPoint p) {
  return cvPoint(p.x - camPosX, camPosY-p.y);
}
CvPoint toImg(CvPoint p) {
  return cvPoint(p.x + camPosX, camPosY-p.y);
}
// CvPoint toCam(CvPoint p) {
//      return cvPoint(p.x - halfresX, halfresY-p.y);
// }
// CvPoint toImg(CvPoint p) {
//      return cvPoint(p.x + halfresX, halfresY-p.y);
// }

void getMask(const CvPoint &pos, float persHeight, float camHeight,
    std::vector<int> &mask)
{
  IplImage *img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
  memset(img->imageData, 0, img->imageSize);
  vector<CvPoint> tpl, hull;
  genTemplate2(pos,persHeight,camHeight,tpl);
  convexHull(tpl,hull);

  CvPoint * corners = new CvPoint[hull.size()];
  for (unsigned i=0; i!=hull.size(); ++i)
    corners[i] = hull[i];
  cvFillConvexPoly(img, corners, hull.size(), CV_RGB(255,255,255));

  mask.resize(width*height);
  unsigned char *src = (unsigned char *)img->imageData;
  unsigned idx=0;
  for (unsigned i=0; i!=height; ++i) {
    unsigned char *s = src;
    for (unsigned j=0; j!=width; ++j,++s,++idx)
      mask[idx] = (*s != 0);
    src += img->widthStep;
  }

  // cvShowImage("image",img);
  // cvWaitKey(0);
  cvReleaseImage(&img);
  delete [] corners;
}

void getMask(const CvPoint &pos, float persHeight, float camHeight,
    std::set<unsigned> &mask)
{
  IplImage *img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
  memset(img->imageData, 0, img->imageSize);
  vector<CvPoint> tpl, hull;
  genTemplate2(pos,persHeight,camHeight,tpl);
  convexHull(tpl,hull);

  CvPoint * corners = new CvPoint[hull.size()];
  for (unsigned i=0; i!=hull.size(); ++i)
    corners[i] = hull[i];
  cvFillConvexPoly(img, corners, hull.size(), CV_RGB(255,255,255));

  mask.clear();
  unsigned char *src = (unsigned char *)img->imageData;
  unsigned idx=0;
  for (unsigned i=0; i!=height; ++i) {
    unsigned char *s = src;
    for (unsigned j=0; j!=width; ++j,++s,++idx)
      if (*s != 0)
        mask.insert(idx);
    src += img->widthStep;
  }

  // cvShowImage("image",img);
  // cvWaitKey(0);
  cvReleaseImage(&img);
  delete [] corners;
}

scanline_t& getElt(vector<scanline_t> &v, unsigned elt)
{
  // cout << "GetElt " << elt << ", size=" << v.size() << endl;
  //      scanline_t s = {0,0,0};
  if (elt >= v.size())
    v.resize(elt+1);
  return v[elt];
}

void getScanLines(vector<CvPoint> &corners, vector<scanline_t> &lines)
{
  int offset = corners[0].y;
  for (unsigned i=0; i!=corners.size(); ++i)
    if (offset>corners[i].y)
      offset = corners[i].y;
  for (unsigned i=0; i+1 < corners.size(); ++i) {
    if (corners[i].y == corners[i+1].y) {
      scanline_t &s = getElt(lines,corners[i].y-offset);
      s.line = corners[i].y;
      if (corners[i].x<corners[i+1].x) {
        s.start = corners[i].x;
        s.end = corners[i+1].x;
      } else {
        s.start = corners[i+1].x;
        s.end = corners[i].x;
      }
    } else if (corners[i].y < corners[i+1].y) {
      float
      x1 = corners[i].x,
      y1 = corners[i].y,
      dx = corners[i+1].x - corners[i].x,
      dy = corners[i+1].y - corners[i].y,
      a = dx/dy;

      for (int y = corners[i+1].y; y>corners[i].y; --y) {
        scanline_t &s = getElt(lines,y-offset);
        s.line = y;
        s.end = (unsigned)(x1 + (y-y1) * a);
        // cout << "line " << s.line << " end " << s.end << endl;
      }
      getElt(lines,corners[i].y-offset).line = corners[i].y;
      getElt(lines,corners[i].y-offset).end = corners[i].x;
      // cout << "line " << getElt(lines,corners[i].y-offset).line
      //      << " end " << getElt(lines,corners[i].y-offset).end << endl;
    } else {
      float
      x1 = corners[i].x,
      y1 = corners[i].y,
      dx = corners[i+1].x - corners[i].x,
      dy = corners[i+1].y - corners[i].y,
      a = dx/dy;

      for (int y = corners[i].y; y>corners[i+1].y; --y) {
        scanline_t &s = getElt(lines,y-offset);
        s.line = y;
        s.start = (unsigned)(x1 + (y-y1) * a);
        // cout << "line " << s.line << " start " << s.start
        //      << ", end= " << s.end << endl;
      }
      getElt(lines,corners[i+1].y-offset).line = corners[i+1].y;
      getElt(lines,corners[i+1].y-offset).start = corners[i+1].x;
      // cout << "line " << getElt(lines,corners[i+1].y-offset).line
      //      << " start " << getElt(lines,corners[i+1].y-offset).start
      //      << endl;
    }
  }
}

void clip(vector<scanline_t> &l)
{
  vector<scanline_t>
  tmp;
  for (unsigned i=0; i!=l.size(); ++i) {
    scanline_t &s = l[i];
    if (s.line >= 0 && s.line < height) {
      if (s.start >= width) {
        if (s.end >= width) {
          s.start = 0;
          s.end = 0;
          continue;
        } else
          s.start = 0;
      }
      if (s.end >= width)
        s.end = width-1;
      tmp.push_back(s);
    }
  }
  l = tmp;
}

void getMask(const vector<CvPoint> &tpl, vector<scanline_t> &mask)
{
  vector<CvPoint> hull;
  convexHull(tpl,hull);
  getScanLines(hull, mask);
  clip(mask);
}

void getMask(const CvPoint &pos, float persHeight, float camHeight,
    vector<scanline_t> &mask)
{
  vector<CvPoint> tpl, hull;
  genTemplate2(pos,persHeight,camHeight,tpl);
  convexHull(tpl,hull);
  getScanLines(hull, mask);
  clip(mask);
}


void listImages(const char *list, std::vector< std::vector<std::string> > &imgs)
{
  ifstream
  ifs(list);
  if (!ifs)
    errx(1, "Cannot open %s", list);

  string dir = list;
  size_t pos = dir.find_last_of('/');
  if (pos != string::npos)
    dir = dir.substr(0,pos) + '/';
  else
    dir = "";


  char buffer[1*1024*1024];
  ifs.getline(buffer, sizeof(buffer));
  while (!ifs.eof()) {
    // cout << "Line: '" << buffer << "'" << endl;
    vector<char *>
    s = splitWhite(buffer, false);
    vector<string>
    line(s.size());
    for (unsigned i=0; i!=s.size(); ++i) // {
      // cout << " ==> '" << s[i] << "'";
      if (s[i][0] != '/') // Relative dir
        line[i] = dir+s[i];
      else             // Absolute dir
        line[i] = s[i];
    //      cout << " --> '" << line[i] << "'";
    // }
    imgs.push_back(line);
    ifs.getline(buffer, sizeof(buffer));
  }
}

void listImages(const char *list, vector<string> &imgs)
{
  ifstream
  ifs(list);
  if (!ifs)
    errx(1, "Cannot open %s", list);

  string dir = list;
  size_t pos = dir.find_last_of('/');
  if (pos != string::npos)
    dir = dir.substr(0,pos) + '/';
  else
    dir = "";

  char buffer[1*1024*1024];
  ifs.getline(buffer, sizeof(buffer));
  while (!ifs.eof()) {
    imgs.push_back((buffer[0]=='/' ? buffer : dir+buffer));
    ifs.getline(buffer, sizeof(buffer));
  }
}

void listImages(const char *list, vector<string> &imgs, vector<double> &tstamps)
{
  ifstream
  ifs(list);
  if (!ifs)
    errx(1, "Cannot open %s", list);

  string dir = list;
  size_t pos = dir.find_last_of('/');
  if (pos != string::npos)
    dir = dir.substr(0,pos) + '/';
  else
    dir = "";

  char buffer[1*1024*1024];
  ifs.getline(buffer, sizeof(buffer));
  while (!ifs.eof()) {
    vector<char *> s = splitWhite(buffer,false);
    if (s.size() == 1) {
      tstamps.push_back(tstamps.size()); // index
      imgs.push_back(dir+s[0]);
    } if (s.size() == 2) {
      tstamps.push_back(atof(s[0]));
      imgs.push_back(dir+s[1]);
    } else if(s.size() == 3) {
      string ts = string(s[0])+" "+ s[1];
      ptime t(time_from_string(ts));
      ptime start(gregorian::date(1970,1,1));
      time_duration diff = t - start;
      // cout << "Time is " << ts << " -> " << to_iso_extended_string(t) << endl;
      tstamps.push_back(double(diff.total_microseconds())/1000000.0);
      imgs.push_back(dir+s[2]);
      // cout << "file " << s[2] << " -> " << imgs.back() << endl;
    } else
      assert("Unknown format");
    // cout << "tstamp=" << s[0] << " - " << setprecision(30) << tstamps.back() << endl;
    ifs.getline(buffer, sizeof(buffer));
  }
}

unsigned char colours [][3] = {
    { 0, 0, 0 },               // unused
    { 255, 0, 0 },             // Blue    1
    { 0, 255, 0 },             // Green   2
    { 0, 0, 255 },             // Red     3
    { 255, 255, 0 },           // Cyan    4
    { 255, 0, 255 },           // magenta 5
    { 0, 255, 255 },           // Yellow  6
    { 0, 128, 255 },           // Orange  7
    // { 255, 255, 255 },         // White   8
    { 128, 0, 0 },             // Blue    9
    { 0, 128, 0 },             // Green   10
    { 0, 0, 128 },             // Red     11
    { 128, 128, 0 },           // Cyan    12
    { 128, 0, 128 },           // magenta 13
    { 0, 128, 128 },           // Yellow  14
    { 0, 64, 128 },            // Orange  15
    { 128, 128, 128 },         // Grey    16
    { 128, 0, 0 },             //
    { 0, 128, 0 },
    { 0, 0, 128 }
};


void applyMask(IplImage *img, const vector<int> &mask)
{
  unsigned char *src = (unsigned char *)img->imageData;
  unsigned index=0;
  for (unsigned i=0; i!=height; ++i) {
    unsigned char *s = src;
    for (unsigned j=0; j!=width; ++j,index++) {
      for (int k=0; k!=channels; ++k,++s)
        if (mask[index]) {
          // *s = colours[15][k];
          if (mask[index] == 2000)
            *s = 255;
          else
            *s = colours[mask[index]][k];
        }
    }
    src += img->widthStep;
  }
}


scanline_t split;
bool _mergeLines(scanline_t &l1, scanline_t &l2, scanline_t *&newLine)
{
  assert(l1.line == l2.line);
  if (l1.start<=l2.start) {
    if (l1.end>=l2.end) { // 1111333331111
      // m.start = l1.start;
      // m.end = l1.end;
      l2.start = l2.end+1; // EMPTY
      return true;
    } else if (l1.end > l2.start) { // 111133332222
      // m.start = v1[i1].start;
      l2.start = l1.end+2;
      l1.end = l2.end;
      return true;
    } else {         // v1.end < v2.start  111111 22222
      return false;
    }
  } else {              // v2.start < v1.start
    if (l2.end>=l1.end) { // 222233332222
      assert(newLine==NULL);
      //                cout << "Can't handle this case...";
      split.start = l1.end+1;
      split.end = l2.end;
      split.line = l2.line;
      newLine = &split;
      l1.end = l2.end;
      l2.end = l1.start-1;
      l1.start = l2.start;
      return true;
    } else if (l2.end+1 >= l1.start) { // 22223333311111
      l2.end = l1.start-1;
      l1.start = l2.start;
      return true;
    } else {         // v2.end < v1.start 2222 111111
      return false;
    }
  }
}

bool overlap(const scanline_t &l1, const scanline_t &l2)
{
  if (l1.line != l2.line)
    return false;

  return (l1.start >= l2.start && l1.start <= l2.end) ||
      (l1.start <= l2.start && l1.end >= l2.end);
}

bool masksOverlap(const vector<scanline_t> &m1, const vector<scanline_t> &m2)
{
  unsigned i=0, j=0;

  while (i!=m1.size() && m1[i].line < m2[0].line)
    i++;
  while (j!=m2.size() && m2[j].line < m1[0].line)
    j++;
  while (j!=m2.size()) {
    while (i!=m1.size() && m1[i].line == m2[j].line)
      if (overlap(m1[i],m2[j]))
        return true;
    ++j;
  }
  return false;
}

void checkMask(const vector<scanline_t> &v)
{
  unsigned line = 0;
  for (unsigned i=0; i!=v.size(); ++i) {
    if (v[i].line < line)
      cout << "Line no " << i << " = " << v[i].line << "< last line " << line << endl;
    line = v[i].line;

    for (unsigned j=i+1; j<v.size(); ++j) {
      if (overlap(v[i],v[j]))
        cout << "Line " << v[i] << " overlaps " << v[j] << endl;
    }
  }
}


/**
 * \brief merge scanlines of v1 and v2
 * \param v1 can already contain merged lines (CONCAVE)
 * \param v2 must be a single, convex mask (CONVEX)
 * 
 * V1 gets to contain the merged mask
 * V2 gets to contain the part of V2 that was not in V2 yet (V2-V2)
 * 
 * 2010/03/25: GWENN - First version
 * 
 **/
void mergeMasksRest(vector<scanline_t> &v1, vector<scanline_t> &v2)
{
  unsigned
  i1=0, i2=0;
  vector<scanline_t>
  merged, rest;

  if (v2.size())
    while (i1 != v1.size() && v1[i1].line < v2[0].line) {
      merged.push_back(v1[i1]);
      ++i1;
    }
  if (v1.size())
    while (i2 != v2.size() && v2[i2].line < v1[0].line) {
      merged.push_back(v2[i2]);
      rest.push_back(v2[i2]);
      ++i2;
    }
  scanline_t
  *n;
  while (i2 != v2.size()) {
    /* check for overlap */
    bool overlap = false;
    n = NULL;
    while (i1 != v1.size() && v1[i1].line==v2[i2].line) {
      if (n) {
        //                     cout << "trimming {" << n->line << "," << n->start << "," << n->end << "}" << endl;
        _mergeLines(v1[i1],*n,n);
        //                     cout << "n = " << *n << endl;
      }
      overlap |= _mergeLines(v1[i1],v2[i2],n);

      merged.push_back(v1[i1]);
      ++i1;
    }
    if (!overlap) {
      merged.push_back(v2[i2]);
      rest.push_back(v2[i2]);
    } else {
      if (v2[i2].start<=v2[i2].end)
        rest.push_back(v2[i2]);
      if (n) {
        //                     cout << "ok" << endl;
        rest.push_back(*n);
      }
    }
    unsigned
    last = merged.size()-2;
    while (last<merged.size() && merged[last].line==merged.back().line) {
      if (_mergeLines(merged[last],merged.back(),n)) {
        merged.pop_back();
        last = merged.size()-2;
      } else
        last--;
    }
    ++i2;
  }

  while (i1 != v1.size()) {
    merged.push_back(v1[i1]);
    ++i1;
  }
  while (i2 != v2.size()) {
    merged.push_back(v2[i2]);
    rest.push_back(v2[i2]);
    ++i2;
  }

  //      checkMask(merged);
  //      checkMask(rest);
  v1 = merged;
  v2 = rest;
  // for (unsigned i=0; i!=merged.size(); ++i) {
  //      for (unsigned j=i+1; j<merged.size(); ++j) {
  //           if (merged[i].line == merged[j].line)
  //                cout << "Dupe for " << merged[i].line << ": 1=[" << merged[i].start << "," << merged[i].end
  //                     << "], 2=[" << merged[j].start << "," << merged[j].end << "]" << endl;
  //      }
  // }
  // cout << endl;
}



bool mergeLines(scanline_t &l1, scanline_t &l2)
{
  assert(l1.line == l2.line);
  if (l1.start<=l2.start) {
    if (l1.end>=l2.end) {
      // m.start = l1.start;
      // m.end = l1.end;
      return true;
    } else if (l1.end+1 >= l2.start) {
      // m.start = v1[i1].start;
      l1.end = l2.end;
      return true;
    } else {         // v1.end < v2.start
      return false;
    }
  } else {              // v2.start < v1.start
    if (l2.end>=l1.end) {
      l1.start = l2.start;
      l1.end = l2.end;
      return true;
    } else if (l2.end+1 >= l1.start) {
      l1.start = l2.start;
      return true;
    } else {         // v1.end < v2.start
      return false;
    }
  }
}

/**
 * \brief merge scanlines of v1 and v2
 * \param v1 can already contain merged lines
 * \param v2 must be a single, convex mask
 * 
 * 2010/03/25: GWENN - First version
 * 
 **/
void mergeMasks(vector<scanline_t> &v1, vector<scanline_t> &v2)
{
  unsigned
  i1=0, i2=0;
  vector<scanline_t>
  merged;

  if (v2.size())
    while (i1 != v1.size() && v1[i1].line < v2[0].line) {
      merged.push_back(v1[i1]);
      ++i1;
    }
  if (v1.size())
    while (i2 != v2.size() && v2[i2].line < v1[0].line) {
      merged.push_back(v2[i2]);
      ++i2;
    }
  while (i2 != v2.size()) {
    /* check for overlap */
    bool overlap = false;
    while (i1 != v1.size() && v1[i1].line==v2[i2].line) {
      overlap |= mergeLines(v1[i1],v2[i2]);
      merged.push_back(v1[i1]);
      ++i1;
      unsigned
      last = merged.size()-1;
      while (last>0 && merged[last-1].line==merged[last].line
          && mergeLines(merged[last-1], merged[last])) {
        merged.pop_back();
        last = merged.size()-1;
      }
    }
    if (!overlap)
      merged.push_back(v2[i2]);
    ++i2;
  }

  while (i1 != v1.size()) {
    merged.push_back(v1[i1]);
    ++i1;
  }
  while (i2 != v2.size()) {
    merged.push_back(v2[i2]);
    ++i2;
  }

  v1 = merged;

  // for (unsigned i=0; i!=merged.size(); ++i) {
  //      for (unsigned j=i+1; j<merged.size(); ++j) {
  //           if (merged[i].line == merged[j].line)
  //                cout << "Dupe for " << merged[i].line << ": 1=[" << merged[i].start << "," << merged[i].end
  //                     << "], 2=[" << merged[j].start << "," << merged[j].end << "]" << endl;
  //      }
  // }
  // cout << endl;
}

float loadPriorHull(const char *file, vector<CvPoint> &polygon)
{
  ifstream
  ifs(file);
  if (!ifs)
    errx(1,"Cannot open file '%s'", file);

  char
  buffer[1024];
  ifs.getline(buffer, sizeof(buffer));
  float
  logInside = atof(buffer);
  ifs.getline(buffer, sizeof(buffer));
  while (!ifs.eof()) {
    vector<char *> s = splitWhite(buffer,true);
    if (s.size() == 2)
      polygon.push_back(cvPoint(atoi(s[0]),atoi(s[1])));
    else {
      cout << __PRETTY_FUNCTION__ <<  "Split size=" << s.size() <<  flush;
      for (unsigned j=0; j!=s.size(); ++j)
        cout << ":" << s[j];
      cout << endl;
    }

    ifs.getline(buffer, sizeof(buffer));

  }
  polygon.push_back(polygon.front());

  return logInside;
}

float loadPrior(const char *file, vector<scanline_t> &prior)
{
  vector<CvPoint>
  polygon;
  float
  logInside = loadPriorHull(file,polygon);
  getScanLines(polygon,prior);
  clip(prior);
  for (unsigned i=0;i!=prior.size(); ++i)
    if (prior[i].start>prior[i].end) {
      cout << "WARNING: scanline start=" << prior[i].start << ", end=" << prior[i].end << " @ line " << prior[i].line << endl;
      FLOAT tmp = prior[i].end;
      prior[i].end = prior[i].start;
      prior[i].start = tmp;
    }
  return logInside;
}

void plotScanLines(IplImage *img, const vector<scanline_t> &mask, const CvScalar &colour, double alpha)
{
  IplImage *blend = img;
  //      IplImage *blend = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
  for (unsigned i=0; i!=mask.size(); ++i) {
    // cvLine(img, cvPoint(mask[i].start,mask[i].line),
    //        cvPoint(mask[i].end,mask[i].line),
    //        colours[i%6][0],colours[i%6][1],colours[i%6][2]),1);
    if (mask[i].start<=mask[i].end)
      cvLine(blend, cvPoint(mask[i].start,mask[i].line),
          cvPoint(mask[i].end,mask[i].line),
          // CV_RGB(0,255,255)
          colour,1);
  }

  // double
  //      beta  = 1-alpha;
  //      cvAddWeighted(img, 1, blend, beta, 0, img);
  //      cvReleaseImage(&blend);
}

void loadPrior(const char *file, vnl_vector<FLOAT> &prior)
{
  vector<scanline_t>
  scan;
  float
  insideProb = loadPrior(file,scan);
  unsigned
  nInside = 0;
  for (unsigned i=0; i!=scan.size(); ++i)
    nInside += scan[i].end - scan[i].start;
  float
  logInside = log(insideProb) - log(nInside),
  logOutside = log(1-insideProb) - log(width*height - nInside);

  prior.set_size(width*height);
  prior.fill(logOutside);
  for (unsigned i=0; i!=scan.size(); ++i) {
    unsigned
    offset = scan[i].line * width;
    for (unsigned j=scan[i].start; j!=scan[i].end;++j)
      prior[offset + j] = logInside;
  }
}

void loadCalibration(const char *fileName)
{
  XmlReader rd(fileName);

  rd.unpack("camPosX", camPosX);
  rd.unpack("camPosY", camPosY);
  rd.unpack("camHeight", camHeight);
  rd.unpack("persHeight", persHeight);
  rd.unpack("w", w);
  w2 = rd.unpackDflt("w2",3*w);
  stdev = rd.unpackDflt("sigma", stdev);
  stdev1 = rd.unpackDflt("sigma1", stdev);
  stdev2 = rd.unpackDflt("sigma2", stdev);
  stdev3 = rd.unpackDflt("sigma3", stdev);
  scanres = rd.unpackDflt("scanres", scanres);
  cerr << "w=" << w << ", w2=" << w2
      << ", scanres=" << scanres
      << ", stdev1=" << stdev1
      << ", stdev2=" << stdev2
      << ", stdev3=" << stdev3
      << endl;

  pixelVar=stdev*stdev, pv2 = 2*pixelVar, lpv2 = .5*log(M_PI * pv2);
  pixelVar1=stdev1*stdev1, pv2_1 = 2*pixelVar1, lpv2_1 = .5*log(M_PI * pv2_1);
  pixelVar2=stdev2*stdev2, pv2_2 = 2*pixelVar2, lpv2_2 = .5*log(M_PI * pv2_2);
  pixelVar3=stdev3*stdev3, pv2_3 = 2*pixelVar3, lpv2_3 = .5*log(M_PI * pv2_3);
}

void saveCalibration(const char *fileName)
{
  XmlWriter
  wr(fileName);

  wr.pack("camPosX", camPosX);
  wr.pack("camPosY", camPosY);
  wr.pack("camHeight", camHeight);
  wr.pack("persHeight", persHeight);
  wr.pack("w", w);
  wr.pack("w2", w2);
  wr.pack("sigma", stdev);
  wr.pack("sigma1", stdev1);
  wr.pack("sigma2", stdev2);
  wr.pack("sigma3", stdev3);
  wr.pack("scanres", scanres);
}


void medianFilter(vnl_vector<FLOAT> &v, unsigned w)
{

}

IplImage *loadImage(const char *filename)
{
  IplImage *res=cvLoadImage(filename);
  if (!res) {
    cerr << "ERROR: Could not load file " << filename << endl;
    abort();
  }
  return res;
}
