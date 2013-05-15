#ifndef Hull_INCLUDED_HH
#define Hull_INCLUDED_HH

#include <vector>
#include <iostream>

struct WorldPoint
{
  double x, y, z;
  
  WorldPoint(double x = 0, double y = 0, double z = 0) :
  x(x), y(y), z(z)
  {
  }
  WorldPoint &operator+=(const WorldPoint &p)
  {
    x += p.x;
    y += p.y;
    z += p.z;
    return *this;
  }
  WorldPoint &operator*=(const double &d)
  {
    x *= d;
    y *= d;
    z *= d;
    return *this;
  }
  WorldPoint &operator/=(const double &d)
  {
    x /= d;
    y /= d;
    z /= d;
    return *this;
  }
  
  double squareDistance(const WorldPoint &p) const
  {
    double dx=x-p.x;
    double dy=y-p.y;
    double dz=z-p.z;
    return dx*dx+dy*dy+dz*dz;
  }
};

std::ostream &operator<<(std::ostream &os, const WorldPoint &wp);
std::ostream &operator<<(std::ostream &os, const std::vector<WorldPoint> &hull);
std::ostream &operator<<(std::ostream &os, const std::vector<std::vector<WorldPoint> > &hulls);

std::vector<char *> splitWhite(char *str, bool dropEmpty);
double sqGroundDist(const WorldPoint &p1, const WorldPoint &p2);

bool inside(const WorldPoint &p, const std::vector<WorldPoint> &hull);
bool inside(const WorldPoint &p, const std::vector<std::vector<WorldPoint> > &hulls);

void saveHull(const char *file, std::vector<WorldPoint> &polygon);
void saveHulls(const char *file, std::vector< std::vector<WorldPoint> >& polygons);
void loadHull(const char *file, std::vector<WorldPoint> &polygon);
void loadHulls(const char *file, std::vector< std::vector<WorldPoint> >& polygons);

#endif
