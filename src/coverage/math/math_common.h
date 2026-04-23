#ifndef __TOOLBOX_MATH_RANDOM_H__
#define __TOOLBOX_MATH_RANDOM_H__

#include <cstdlib>
#include <ctime>

#include "coverage/geometry/geoheader.h"

using geometry::Site;
using geometry::SiteVec;

namespace math {

class Math {
 public:
  Math() { std::srand((unsigned)std::time(nullptr)); }

  ~Math() {}

 public:
  /* *
   * @brief  this is for geting a random number ar or degree
   * @param  min: the lower bound of random number
   *         max: the upper bound of random number
   * @return a random number between [min, max]
   * */
  double rand(const double min, const double max) const;

  double rand_ac(void) const { return rand(0, M_PI); }

  double rand_degree(void) const { return rand(0, 180); }

  /* *
   * @brief  get a random site in a circle with specific center and radius
   * @param  center: the center of the circle
   *         radius: the radius of the circle
   * @return a random site in the circle
   * */
  Site rand(const Site &center, const double radius) const;

  /* *
   * @brief  get the center and radius by three point
   * @param  p1,p2,p3: three specific point
   * @return the center and radius of the circle
   * */
  Site Circle(const Site p1, const Site p2, const Site p3, double &radius);

  /**
   * @brief   get a arc path
   * @param   center: the center of the arc
   *          radius: the radius of the arc
   *          base:   the start of the arc
   *          length: the length of the arc
   * @return  path point list
   **/
  void GetArc(SiteVec &list, Site &center, double &radius, Site &base,
              double length = 0.8);

  /* *
   * @brief  to interpolate a vector
   * @param  list    vector to be interpolated
   * @param  dens    density of interpolation
   * @return   0 success
   * @return  -1 failure
   * */
  int Interpolate(SiteVec &list, const double dens = 0.05);
};

}  // namespace math

#endif  // __TOOLBOX_MATH_RANDOM_H__
