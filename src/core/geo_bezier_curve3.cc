
#include "core/geo_bezier_curve3.h"

#include <algorithm>

#include "algorithm/mc_math.h"

namespace geditor {

GeoBezierCurve3::GeoBezierCurve3() : Geometry(GT_BEZIER_CURVE) {}

GeoBezierCurve3::~GeoBezierCurve3() {}

double bezier3funcX(double uu, Point3d *controlP) {
  double part0 = controlP[0].x * uu * uu * uu;
  double part1 = 3 * controlP[1].x * uu * uu * (1 - uu);
  double part2 = 3 * controlP[2].x * uu * (1 - uu) * (1 - uu);
  double part3 = controlP[3].x * (1 - uu) * (1 - uu) * (1 - uu);
  return part0 + part1 + part2 + part3;
}

double bezier3funcY(double uu, Point3d *controlP) {
  double part0 = controlP[0].y * uu * uu * uu;
  double part1 = 3 * controlP[1].y * uu * uu * (1 - uu);
  double part2 = 3 * controlP[2].y * uu * (1 - uu) * (1 - uu);
  double part3 = controlP[3].y * (1 - uu) * (1 - uu) * (1 - uu);
  return part0 + part1 + part2 + part3;
}

void bezierCurveCreate(Point3d *controlPoint, std::vector<Point3d> &curvePoint,
                       double step) {
  double u = 1;
  while (u >= 0) {
    double px = bezier3funcX(u, controlPoint);
    double py = bezier3funcY(u, controlPoint);
    u -= step;
    Point3d tempP = Point3d(px, py, 0.0);
    curvePoint.push_back(tempP);
  }
}

void createCurve(Point3d *originPoint, int originCount,
                 std::vector<Point3d> &curvePoint) {
  float scale = 0.6f;
  Point3d *midpoints = new Point3d[originCount];
  for (int i = 0; i < originCount; i++) {
    int nexti = (i + 1) % originCount;
    midpoints[i].x = (originPoint[i].x + originPoint[nexti].x) / 2.0;
    midpoints[i].y = (originPoint[i].y + originPoint[nexti].y) / 2.0;
  }
  Point3d *extrapoints = new Point3d[2 * originCount];
  for (int i = 0; i < originCount; i++) {
    int nexti = (i + 1) % originCount;
    int backi = (i + originCount - 1) % originCount;
    Point3d midinmid;
    midinmid.x = (midpoints[i].x + midpoints[backi].x) / 2.0;
    midinmid.y = (midpoints[i].y + midpoints[backi].y) / 2.0;
    double offsetx = originPoint[i].x - midinmid.x;
    double offsety = originPoint[i].y - midinmid.y;
    int extraindex = 2 * i;
    extrapoints[extraindex].x = midpoints[backi].x + offsetx;
    extrapoints[extraindex].y = midpoints[backi].y + offsety;
    double addx = (extrapoints[extraindex].x - originPoint[i].x) * scale;
    double addy = (extrapoints[extraindex].y - originPoint[i].y) * scale;
    extrapoints[extraindex].x = originPoint[i].x + addx;
    extrapoints[extraindex].y = originPoint[i].y + addy;

    int extranexti = (extraindex + 1) % (2 * originCount);
    extrapoints[extranexti].x = midpoints[i].x + offsetx;
    extrapoints[extranexti].y = midpoints[i].y + offsety;
    addx = (extrapoints[extranexti].x - originPoint[i].x) * scale;
    addy = (extrapoints[extranexti].y - originPoint[i].y) * scale;
    extrapoints[extranexti].x = originPoint[i].x + addx;
    extrapoints[extranexti].y = originPoint[i].y + addy;
  }

  Point3d *controlPoint = new Point3d[4];
  for (int i = 0; i < originCount - 1; i++) {
    controlPoint[0] = originPoint[i];

    int extraindex = 2 * i;
    controlPoint[1] = extrapoints[extraindex + 1];

    int extranexti = (extraindex + 2) % (2 * originCount);
    controlPoint[2] = extrapoints[extranexti];

    int nexti = (i + 1) % originCount;
    controlPoint[3] = originPoint[nexti];

    if (originCount == 2) {
      // u�Ĳ����������ߵ�����  �տ�ʼ��0.005Ҳ����200����
      // u -= 0.05;
      bezierCurveCreate(controlPoint, curvePoint, 0.025);
    }
    if (originCount > 2) {
      double alpha = 0;
      if (i + 2 != originCount) {
        Point3d ptv1 = originPoint[i + 1] - originPoint[i];
        Point3d ptv2 = originPoint[i + 2] - originPoint[i + 1];

        V3d v1 = V3d(ptv1.x, ptv1.y, ptv1.z);
        V3d v2 = V3d(ptv2.x, ptv2.y, ptv2.z);

        double molecular = v1.dot(v2);
        double denominator = v1.norm() * v2.norm();
        double cosvalue = molecular / denominator;
        alpha = acos(cosvalue) * 180 / M_PI;
      }
      double stdalpha = 180 - alpha;
      if (stdalpha >= 150 && stdalpha <= 180) {
        bezierCurveCreate(controlPoint, curvePoint, 0.02);
      }
      if (stdalpha >= 120 && stdalpha < 150) {
        bezierCurveCreate(controlPoint, curvePoint, 0.05);
      }
      if (stdalpha >= 90 && stdalpha < 120) {
        bezierCurveCreate(controlPoint, curvePoint, 0.025);
      }
      if (stdalpha >= 60 && stdalpha < 90) {
        bezierCurveCreate(controlPoint, curvePoint, 0.0125);
      }
      if (stdalpha >= 30 && stdalpha < 60) {
        bezierCurveCreate(controlPoint, curvePoint, 0.00625);
      }
      if (stdalpha >= 0 && stdalpha < 30) {
        bezierCurveCreate(controlPoint, curvePoint, 0.005);
      }
    }
  }
}

void GeoBezierCurve3::Hermite(std::vector<Point3d> &items) {
  int originCount = m_pointSet.size();
  items.clear();
  createCurve(&m_pointSet[0], originCount, items);
}

int GeoBezierCurve3::IsPointInEdge(const Point3d &point, Point3d &outPnt,
                                   double tolerance, double &fal) {
  int nSize = m_pointSet.size();
  if (nSize > 1) {
    double minLength = 555555;
    int findIndex = 0;
    Point3d nearPnt;

    std::vector<Point3d> items;
    Hermite(items);

    if (items.size() > 1) {
      Point3d start = items[0];

      for (int j = 1; j < items.size(); j++) {
        Point3d end = items[j];
        Point3d pt3d;
        double length = OnSegment(start, end, point, pt3d);
        if (minLength > length) {
          nearPnt = pt3d;
          findIndex = 1;
          minLength = length;
        }

        start = end;
      }
    }

    if (tolerance > minLength) {
      outPnt = nearPnt;
      fal = tolerance;
      return findIndex;
    }

  } else if (nSize > 0) {
    Point3d start = m_pointSet[0];

    double length = sqrt((start.x - point.x) * (start.x - point.x) +
                         (start.y - point.y) * (start.y - point.y) +
                         (start.z - point.z) * (start.z - point.z));
    if (length < tolerance) {
      fal = length;
      outPnt = start;
      return 1;
    }
  }
  return false;
}

void GeoBezierCurve3::MoveGeometry(double x, double y, double z) {
  int nSize = m_pointSet.size();
  if (nSize > 1) {
    for (int i = 0; i < nSize; i++) {
      Point3d end = m_pointSet[i];
      m_pointSet[i].x = end.x + x;
      m_pointSet[i].y = end.y + y;
      m_pointSet[i].z = end.z + z;
    }
  }
  m_bDirty = true;
}

double GeoBezierCurve3::Length() const {
  double dSum = 0.0;

  int nSize = m_pointSet.size();
  if (nSize > 1) {
    Point3d start = m_pointSet[0];
    for (int i = 0; i < nSize; i++) {
      Point3d end = m_pointSet[i];
      dSum += sqrt((start.x - end.x) * (start.x - end.x) +
                   (start.y - end.y) * (start.y - end.y) +
                   (start.z - end.z) * (start.z - end.z));
      start = end;
    }
  }
  return dSum;
}

double GeoBezierCurve3::GetNeartPoint(const Point3d &point) const {
  Point3d retPoint;

  int iCount = m_pointSet.size();
  if (iCount > 1) {
    double distance = 2000.0;
    Point3d segStart;
    Point3d segEnd;

    Point3d start = m_pointSet[0];
    for (int i = 1; i < iCount; i++) {
      Point3d end = m_pointSet[i];

      Point3d outPt;
      double dist = OnSegment(start, end, point, outPt);
      if (dist < distance) {
        segStart = start;
        segEnd = end;

        distance = dist;
        retPoint = outPt;
      }

      start = end;
    }

    Point3d vec1(segEnd.x - segStart.x, segEnd.y - segStart.y,
                 segEnd.z - segStart.z);
    Point3d vec2(point.x - retPoint.x, point.y - retPoint.y,
                 point.z - retPoint.z);

    double dCross = vec1.x * vec2.y - vec1.y * vec2.x;

    return dCross;
  }

  return 0.0;
}

int GeoBezierCurve3::OnPoint(const Point3d &Q, double tolerance) {
  int index = -1;
  double minFlag = 100;

  int nSize = m_pointSet.size();
  for (int i = 0; i < nSize; i++) {
    double maxVal = Mathd::Max(Mathd::Abs(m_pointSet[i].x - Q.x),
                               Mathd::Abs(m_pointSet[i].y - Q.y));
    if (maxVal < minFlag) {
      minFlag = maxVal;
      index = i;
    }
  }

  if (minFlag < tolerance) {
    return index;
  } else {
    return -1;
  }
}

const void *GeoBezierCurve3::GetDataPtr() const { return &(m_pointSet[0]); }

const int GeoBezierCurve3::GetDataSize() const {
  size_t count = m_pointSet.size();
  return count * sizeof(Point3d);
}

}  // namespace geditor
