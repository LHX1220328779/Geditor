
#include "core/geometry.h"

#include <algorithm>
#include <limits>

namespace geditor {

Geometry::Geometry(GeometryType type) : m_geoType(type), m_bDirty(true) {}

Geometry::~Geometry() {}

Geometry::GeometryType Geometry::GetGeometryType() const { return m_geoType; }

void Geometry::ReverseVertex() {
  std::reverse(m_pointSet.begin(), m_pointSet.end());
  m_bDirty = true;
  OnChange();
}

int Geometry::GetVertexCount() { return m_pointSet.size(); }

Point3d Geometry::GetVertex(int index) const { return m_pointSet[index]; }

Point3d *Geometry::GetStartVertexPtr() {
  int iCount = m_pointSet.size();
  if (iCount > 0) {
    return &(m_pointSet[0]);
  } else {
    return nullptr;
  }
}

Point3d *Geometry::GetEndVertexPtr() {
  int iCount = m_pointSet.size();
  if (iCount > 0) {
    return &(m_pointSet[iCount - 1]);
  } else {
    return nullptr;
  }
}

Point3d Geometry::GetStartVertex() const {
  int iCount = m_pointSet.size();
  if (iCount > 0) {
    return m_pointSet[0];
  } else {
    return Point3d();
  }
}

Point3d Geometry::GetEndVertex() const {
  int iCount = m_pointSet.size();
  if (iCount > 0) {
    return m_pointSet[iCount - 1];
  } else {
    return Point3d();
  }
}

void Geometry::AppendVertex(double x, double y, double z) {
  m_pointSet.push_back(Point3d::Point3dId(x, y, z));
  m_bDirty = true;

  OnChange();
}

void Geometry::AppendVertex(const Point3d &point) {
  Point3d p = point;
  p.GetId();
  m_pointSet.push_back(p);
  m_bDirty = true;

  OnChange();
}

void Geometry::MoveVertex(const Point3d &point, int index) {
  if (index < m_pointSet.size()) {
    m_pointSet[index].x = point.x;
    m_pointSet[index].y = point.y;
    m_pointSet[index].z = point.z;
    m_bDirty = true;

    OnChange();
  }
}

void Geometry::OffsetVertex(const Point3d &point, int index) {
  if (index < m_pointSet.size()) {
    m_pointSet[index].x += point.x;
    m_pointSet[index].y += point.y;
    m_pointSet[index].z += point.z;

    OnChange();

    m_bDirty = true;
  }
}

void Geometry::MoveGeometry(double x, double y, double z) {
  MoveGeometry(Point3d(x, y, z));
}

void Geometry::MoveGeometry(const Point3d &vDir) {
  for (int index = 0; index < m_pointSet.size(); index++) {
    m_pointSet[index].x += vDir.x;
    m_pointSet[index].y += vDir.y;
    m_pointSet[index].z += vDir.z;
  }
  OnChange();
  m_bDirty = true;
}

void Geometry::InsertVertex(int index, const Point3d &point) {
  Point3d p = point;
  p.GetId();
  if (index < m_pointSet.size()) {
    m_pointSet.insert(m_pointSet.begin() + index, p);
  } else {
    m_pointSet.push_back(p);
  }
  OnChange();
  m_bDirty = true;
}

void Geometry::RemoveVertex(int index) {
  if (index >= 0 && index < m_pointSet.size()) {
    m_pointSet.erase(m_pointSet.begin() + index);

    m_bDirty = true;

    OnChange();
  }
}

void Geometry::Resize(int nsize) {
  if (nsize != m_pointSet.size()) {
    m_pointSet.resize(nsize, Point3d());
    m_bDirty = true;

    OnChange();
  }
}

bool Geometry::IsVaild() const {
  if (m_pointSet.size() > 1) {
    return true;
  } else {
    return false;
  }
}

void Geometry::CalculateBoundBox() {
  double dMax = (std::numeric_limits<double>::max)();

  m_boundBox.Reset();

  for (size_t i = 0; i < m_pointSet.size(); i++) {
    m_boundBox.ExpandBy(m_pointSet[i].x, m_pointSet[i].y, m_pointSet[i].z);
  }

  // m_boundBox.vMin.z = -1000;
  // m_boundBox.vMax.z = 1000;

  m_bDirty = false;
}

bool Geometry::IsBoundDirty() { return m_bDirty; }

void Geometry::SetBoundDirty(bool bDirty) { m_bDirty = bDirty; }

BoundBox3d Geometry::GetBound() { return m_boundBox; }

bool FindValue(std::list<int> &pointIndexsToKeep, int indexFarthest) {
  std::list<int>::iterator iter;
  for (iter = pointIndexsToKeep.begin(); iter != pointIndexsToKeep.end();
       ++iter) {
    if (*iter == indexFarthest) {
      return true;
    }
  }
  return false;
}

double Geometry::PerpendicularDistance(Point3d Point1, Point3d Point2,
                                       Point3d Point3) {
  double area = abs(0.5 * (Point1.x * Point2.y + Point2.x * Point3.y +
                           Point3.x * Point1.y - Point2.x * Point1.y -
                           Point3.x * Point2.y - Point1.x * Point3.y));
  double bottom =
      sqrt(pow(Point1.x - Point2.x, 2) + pow(Point1.y - Point2.y, 2));
  double height = area / bottom * 2;

  return height;
}

void Geometry::DouglasPeuckerReduction(std::vector<Point3d> points,
                                       int firstPoint, int lastPoint,
                                       double tolerance,
                                       std::list<int> &pointIndexsToKeep) {
  double maxDistance = 0;
  int indexFarthest = 0;

  for (int index = firstPoint + 1; index < lastPoint; index++) {
    double distance = PerpendicularDistance(points[firstPoint],
                                            points[lastPoint], points[index]);
    if (distance > maxDistance) {
      maxDistance = distance;
      indexFarthest = index;
    }
  }

  if (maxDistance > tolerance && indexFarthest != 0) {
    //�Ƿ������е�list�����ҵ�����ͬ��ֵ������������������������
    bool bFlag = FindValue(pointIndexsToKeep, indexFarthest);
    if (!bFlag) {
      pointIndexsToKeep.push_back(indexFarthest);
    }
    DouglasPeuckerReduction(points, firstPoint, indexFarthest, tolerance,
                            pointIndexsToKeep);

    DouglasPeuckerReduction(points, indexFarthest, lastPoint, tolerance,
                            pointIndexsToKeep);
  }
}

Geometry *Geometry::DouglasPeucker(Geometry *pGeometry, double Tolerance) {
  std::vector<Point3d> Points = pGeometry->m_pointSet;
  if (Points.empty() || (Points.size() < 3)) {
    return NULL;
  }

  int firstPoint = 0;
  int lastPoint = Points.size() - 1;
  std::list<int> pointIndexsToKeep;

  //����һ��������һ������ӵ�������������
  pointIndexsToKeep.push_back(firstPoint);
  pointIndexsToKeep.push_back(lastPoint);

  //��һ��������һ���㲻���ظ�
  while (Points[firstPoint] == Points[lastPoint]) {
    lastPoint--;
  }

  DouglasPeuckerReduction(Points, firstPoint, lastPoint, Tolerance,
                          pointIndexsToKeep);

  std::vector<Point3d> resPoints;
  pointIndexsToKeep.sort();
  std::list<int>::iterator theIterator;
  for (theIterator = pointIndexsToKeep.begin();
       theIterator != pointIndexsToKeep.end(); theIterator++) {
    resPoints.push_back(Points[*theIterator]);
  }
  pGeometry->Reset();
  pGeometry->m_pointSet = resPoints;

  return pGeometry;
}

void Geometry::Reset() {
  m_bDirty = true;
  m_pointSet.clear();
}

void Geometry::OnChange() {}

int Geometry::CopyGeometry(double offset, Geometry *line) {
  int iCount = m_pointSet.size();
  if (iCount < 2 || !line) return -1;
  Point3d v;
  for (int i = 0; i < iCount - 1; i++) {
    double dis = Point3d::Distance(m_pointSet[i + 1], m_pointSet[i]);
    if (dis < 0.01) continue;
    v = m_pointSet[i + 1] - m_pointSet[i];
    v /= dis;
    Point3d dxy(-v.y * offset, v.x * offset, 0);
    Point3d p = m_pointSet[i];
    p += dxy;
    line->AppendVertex(p);
  }
  Point3d dxy(-v.y * offset, v.x * offset, 0);
  Point3d p = m_pointSet[iCount - 1];
  p += dxy;
  line->AppendVertex(p);

  return 0;
}

double Geometry::GetNeartPoint(const Point3d &point, Point3d &out) const {
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
    out = retPoint;
    return dCross;
  }

  return 0.0;
}
double Geometry::GetNeartPoint(const Point3d &point) const {
  Point3d out;
  return GetNeartPoint(point, out);
}

double Geometry::OnSegment(const Point3d &P1, const Point3d &P2,
                           const Point3d &pt, Point3d &outPt) const {
  double fDot = (P2.x - P1.x) * (pt.x - P1.x) + (P2.y - P1.y) * (pt.y - P1.y);
  if (fDot <= 0.0f) {
    outPt = P1;
    return sqrt((P1.x - pt.x) * (P1.x - pt.x) + (P1.y - pt.y) * (P1.y - pt.y));
  }

  double d2AB = (P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y);
  if (fDot >= d2AB) {
    outPt = P2;
    return sqrt((P2.x - pt.x) * (P2.x - pt.x) + (P2.y - pt.y) * (P2.y - pt.y));
  }

  double u = fDot / d2AB;

  outPt.x = P1.x + (P2.x - P1.x) * u;
  outPt.y = P1.y + (P2.y - P1.y) * u;
  outPt.z = 0.0;

  return sqrt((pt.x - outPt.x) * (pt.x - outPt.x) +
              (pt.y - outPt.y) * (pt.y - outPt.y));
}

void Geometry::Hermite(std::vector<Point3d> &items) { items = m_pointSet; }

int Geometry::IsPointInEdge(const Point3d &point, Point3d &outPnt,
                            double tolerance, double &fal) {
  int nSize = m_pointSet.size();
  if (nSize > 1) {
    BoundBox3d box(m_boundBox);
    box.Extend(tolerance);

    if (box.Contains(V3d(point.x, point.y, point.z))) {
      Point3d start = m_pointSet[0];

      for (int i = 1; i < nSize; i++) {
        Point3d end = m_pointSet[i];

        double length = OnSegment(start, end, point, outPnt);
        if (length < tolerance) {
          fal = length;
          return i;
        }

        start = end;
      }
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

  return 0;
}

double Geometry::Length() const {
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
}  // namespace geditor
