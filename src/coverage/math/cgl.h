#ifndef INCLUDE_MATH_CGL_H_
#define INCLUDE_MATH_CGL_H_

#include "coverage/geometry/geoheader.h"

namespace math {
namespace cgl {

using geometry::Site;
using geometry::SiteVec;

/**
 * @brief : get the cross product from p1 to p2
 * @param : none
 * @return: p1 x p2
 **/
double CrossProduct(const Site &p1, const Site &p2);

/**
 * @brief : get the cross product from p1p2 to p3p4
 * @param : none
 * @return: p1p2 x p3p4
 **/
double CrossProduct(const Site &p1, const Site &p2, const Site &p3,
                    const Site &p4);

/**
 * @brief : get the cross product from pp1 to pp2
 * @param : none
 * @return: pp1 x pp2
 **/
double CrossProduct(const Site &p, const Site &p1, const Site &p2);

/**
 * @brief : get the dot product from p1 to p2
 * @param : none
 * @return: p1 x p2
 **/
double DotProduct(const Site &p1, const Site &p2);

/**
 * @brief : get the dot product from p1p2 to p3p4
 * @param : none
 * @return: p1p2 . p3p4
 **/
double DotProduct(const Site &p1, const Site &p2, const Site &p3,
                  const Site &p4);

/**
 * @brief : get the dot product from pp1 to pp2
 * @param : none
 * @return: pp1 . pp2
 **/
double DotProduct(const Site &p, const Site &p1, const Site &p2);

/**
 * @brief : check the poly is in the autoclockwise order
 * @param : none
 * @return: if the poly is in the autoclockwise order, return true;
 *          otherwise, return false;
 **/
bool IfAutoclockwise(const SiteVec &poly);

/**
 * @brief : check two lines are in the same direction or the reverse direction
 * @param : none
 * @return: return true if they are in the same direction
 * @others: please make sure two line is parallel
 **/
bool SameDirection(const Site &p1, const Site &p2);

/**
 * @brief : check two lines are parallel
 * @param : none
 * @return: return true if they are parallel
 * @others: please make sure two line is parallel
 **/
bool IsParallel(const Site &p1, const Site &p2);

/**
 * @brief : check two segments p1p2 and p3p4 is in cross
 * @param : none
 * @return: return true if they are in cross
 **/
bool CheckCrossOfSegments(const Site &p1, const Site &p2, const Site &p3,
                          const Site &p4);

/**
 * @brief : check the point p is on the left of the line p1p2
 * @param : none
 * @return: return true if p is on the left,
 *          return false if p is on the right.
 **/
bool CheckLeftOfEdge(const Site &p, const Site &p1, const Site &p2);

/**
 * @brief : get the cross point of two segments p1p2 and p3p4
 * @param : none
 * @return: return true if p1p2 and p3p4 have the cross points,
 *          return false if p1p2 and p3p4 have no cross points.
 * @others: p1p2 and p3p4 are two segments
 **/
bool GetCrossPtOfSegments(const Site &p1, const Site &p2, const Site &p3,
                          const Site &p4, Site &p);

/**
 * @brief : get the cross point of two lines p1p2 and p3p4
 * @param : none
 * @return: return true if p1p2 and p3p4 have the cross points,
 *          return false if p1p2 and p3p4 are parallel.
 * @others: p1p2 and p3p4 are two lines
 **/
bool GetCrossPtOfLines(const Site &p1, const Site &p2, const Site &p3,
                       const Site &p4, Site &p);

/**
 * @brief : get the distance from pt to the line p1p2
 * @param : none
 * @return: return the distance
 **/
double GetDisFromPtToLine(const Site &pt, const Site &p1, const Site &p2);

/**
 * @brief : check the point is inside the polygon
 * @param : none
 * @return: return true if the point pt is inside the polygon;
 *          return false otherwise.
 **/
bool IsInside(const Site &pt, const SiteVec &polygon);

/**
 * @brief : get the gravity of a triangle
 * @param : none
 * @return:
 * @others:
 *��ȡ���ģ������ߵ����߽���һ�㣩
 **/
Site GravityPt(const Site &p1, const Site &p2, const Site &p3);

/**
 * @brief : get the IncenterPt of a triangle
 * @param : none
 * @return:
 * @others:
 *��ȡ���ģ������ڽ�ƽ���߽���һ�㣩������Բ��ԭԲ�ģ�
 **/
Site IncenterPt(const Site &p1, const Site &p2, const Site &p3);

/**
 * @brief : get the CircumPt of a triangle
 * @param : none
 * @return:
 * @others:
 *��ȡ���ģ������ߵĴ�ֱƽ���߽���һ�㣩�����Բ��Բ�ģ�
 **/
Site CircumPt(const Site &a, const Site &b, const Site &c);

/**
 * @brief : get the OrthoPt of a triangle
 * @param : none
 * @return:
 * @others:
 *��ȡ���ģ������ε������߽���һ�㣩
 **/
Site OrthoPt(const Site &a, const Site &b, const Site &c);

int CheckCrossSeg2Poly(const Site &p1, const Site &p2, const SiteVec &poly,
                       int &ind, double &dis);

/// �㵽����ξ���
double DisFromPt2Poly(const Site &p, const SiteVec &poly);

double AreaOfPoly(const SiteVec &poly);
}  // namespace cgl
}  // namespace math

#endif  // INCLUDE_MATH_CGL_H_
