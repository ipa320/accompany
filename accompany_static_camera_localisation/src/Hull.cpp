#include <Hull.h>

#include <Helpers.hh>

#include <fstream>
#include <math.h>
#include <err.h>
using namespace std;

ostream &operator<<(ostream &os, const WorldPoint &wp)
{
  return os << "(" << wp.x << "," << wp.y << "," << wp.z << ")";
}

double sqGroundDist(const WorldPoint &p1, const WorldPoint &p2)
{
  double dx = p1.x - p2.x, dy = p1.y - p2.y; // ignore z

  return dx * dx + dy * dy;
}

bool inside(const WorldPoint &p, const vector<WorldPoint> &prior)
{
  unsigned nLeft = 0, nRight = 0;
  for (unsigned i = 1; i < prior.size(); ++i)
  {
    if ((p.y >= prior[i - 1].y && p.y < prior[i].y)
        || (p.y >= prior[i].y && p.y < prior[i - 1].y))
    {
      double deltaX = prior[i].x - prior[i - 1].x, deltaY = prior[i].y
          - prior[i - 1].y;
      if (fabs(deltaX) > fabs(deltaY))
      {
        double a = deltaY / deltaX, b = prior[i].y - a * prior[i].x, x = (p.y
            - b) / a;
        if (x < p.x)
          nLeft++;
        else
          // on the line is out
          nRight++;
      }
      else
      {
        double a = deltaX / deltaY, b = prior[i].x - a * prior[i].y, x = a * p.y
            + b;
        if (x < p.x)
          nLeft++;
        else
          // on the line is out
          nRight++;
      }
    }
  }
  return (nLeft % 2 == 1 && nRight % 2 == 1);
}

/**
 * \brief Split a (CSV-separated) string
 * \param str The string to split
 * \param dropEmpty whether to drop empty fields
 * \return A vector of strings, containing the fields
 * 
 * WARNING: for efficiency, the original string is destroyed
 * 
 * No copy operations are performed in this function.
 * 
 * 2008/05/07: GWENN - First version
 * 
 **/
std::vector<char *> splitWhite(char *str, bool dropEmpty)
{
  std::vector<char *>
    res;
  char
    *prev = str;
  while (*str) {
    if (isspace(*str)) {
      if (!dropEmpty || str!=prev)
        res.push_back(prev);
      if (dropEmpty)  {
        while (isspace(*str)) {
          *str = '\0';
          ++str;
        }
      } else {
        *str = '\0';
        ++str;
      }
      prev = str;
    } else
      str++;
  }
  if (!dropEmpty || str!=prev)
    res.push_back(prev);

  return res;
}

bool same(const WorldPoint& p1,const WorldPoint& p2)
{
  double diff;
  diff=p1.x-p2.x;
  if (diff*diff>0.001)
    return false;
   diff=p1.y-p2.y;
  if (diff*diff>0.001)
    return false;
   diff=p1.z-p2.z;
  if (diff*diff>0.001)
    return false;
  return true;
}

void saveHull(ofstream& outfile, std::vector<WorldPoint>& polygon)
{
  for (unsigned i=0; i!=polygon.size(); ++i)
  {
    if (i==0 || !same(polygon[i-1],polygon[i])) // avoid saving same WorldPoint twice as loadHull adds a worldpoint
        outfile << polygon[i].x << " " << polygon[i].y << " " << polygon[i].z << endl;
  }
  outfile<<endl;
}

void saveHull(const char *file, std::vector<WorldPoint>& polygon)
{
  ofstream outfile;
  outfile.open(file);
  saveHull(outfile,polygon);
  outfile.close();
}

void saveHulls(const char *file, std::vector< std::vector<WorldPoint> >& polygons)
{
  ofstream outfile;
  outfile.open(file);
  for (unsigned i=0;i<polygons.size();i++)
    saveHull(outfile,polygons[i]);
  outfile.close();
}

void loadHull(ifstream& ifs, vector<WorldPoint>& polygon)
{
  char buffer[1024];
  ifs.getline(buffer, sizeof(buffer));
  while (!ifs.eof())
  {
    vector<char *> s = splitWhite(buffer, true);
    
    if (s.size() == 3)
      polygon.push_back(WorldPoint(atoi(s[0]), atoi(s[1]), atoi(s[2])));
    else if (s.size() == 2)
      polygon.push_back(WorldPoint(atoi(s[0]), atoi(s[1]), 0));
    else if (s.size() == 0)
      break;
    else
    {
      cout << "ERROR " << __PRETTY_FUNCTION__ << " Split size=" << s.size() << flush;
      for (unsigned j = 0; j != s.size(); ++j)
        cout << ":" << s[j];
      cout << endl;
    }
    ifs.getline(buffer, sizeof(buffer));
  }
  if (polygon.size()>0)
    polygon.push_back(polygon.front());
}

void loadHull(const char *file, vector<WorldPoint>& polygon)
{
  ifstream ifs(file);
  if (!ifs) 
    warnx("Cannot open file '%s'", file);
  else
  {
    loadHull(ifs,polygon);
    ifs.close();
  }
}

void loadHulls(const char *file, vector< vector<WorldPoint> >& polygons)
{
  ifstream ifs(file);
  if (!ifs)
    warnx("Cannot open file '%s'", file);
  else
  {
    while(true)
    {
      vector<WorldPoint> pol;
      loadHull(ifs,pol);
      if (pol.size()>0)
        polygons.push_back(pol);
      else
        break;
    }
    ifs.close();
  }
}
