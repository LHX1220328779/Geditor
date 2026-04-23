
#pragma once

#include <list>
#include <vector>

#include "algorithm/bound_box.h"
#include "core/point3d.h"
#include "core/point_color.h"

namespace geditor {

class Geometry {
 public:
  enum GeometryType {
    GT_UNKOWN_TYPE = 0,
    GT_POLYLINE = 1,
    GT_POLYGON = 2,
    GT_BEZIER_CURVE = 3,
    GT_BSPLINE_CURVE = 4,
    GT_HERMITE_CURVE = 5,
    GT_CIRCULAR_ARC = 6,
    GT_POINT = 7,
    GT_RECTANGLE = 8,
    GT_POLYGON_HOLE = 9,
    GT_ARC_LINE = 10,
  };

 public:
  Geometry(GeometryType type);

  virtual ~Geometry();

 public:
  GeometryType GetGeometryType() const;

  virtual void Hermite(std::vector<Point3d> &items);

  virtual void MoveVertex(const Point3d &point, int index);

  virtual void OffsetVertex(const Point3d &point, int index);

  virtual void AppendVertex(double x, double y, double z);

  virtual void AppendVertex(const Point3d &point);

  virtual void InsertVertex(int index, const Point3d &point);

  virtual void RemoveVertex(int index);

  virtual void MoveGeometry(double x, double y, double z);

  virtual void MoveGeometry(const Point3d &vDir);

  virtual Point3d GetVertex(int index) const;

  virtual int CopyGeometry(double offset, Geometry *line);

  virtual double GetNeartPoint(const Point3d &point) const;

  virtual double GetNeartPoint(const Point3d &point, Point3d &out) const;

  virtual double OnSegment(const Point3d &P1, const Point3d &P2,
                           const Point3d &pt, Point3d &outPt) const;
  virtual int IsPointInEdge(const Point3d &point, Point3d &outPnt,
                            double tolerance, double &fal);
  virtual double Length() const;

  void Resize(int nsize);

  int GetVertexCount();

  void ReverseVertex();

  virtual bool IsVaild() const;

  void CalculateBoundBox();

  bool IsBoundDirty();

  Point3d GetStartVertex() const;

  Point3d GetEndVertex() const;

  Point3d *GetStartVertexPtr();

  Point3d *GetEndVertexPtr();

  BoundBox3d GetBound();

  //! \brief �㵽ֱ�߾��뺯����
  //! \param Point1 [in] ��һ���㡣
  //! \param Point2 [in] �ڶ����㡣
  //! \param Point3 [in] �������㡣
  //! \remarks ����Point1��Point2��Point3��ȡPoint3��Point1��Point2ֱ�ߵľ��롣
  double PerpendicularDistance(Point3d Point1, Point3d Point2, Point3d Point3);

  //! \brief ��ԭʼ�㼯��ȡ�µĵ㼯��
  //! \param points [in] ԭʼ�㼯��
  //! \param firstPoint [in] ��һ�����������
  //! \param lastPoint [in] �ڶ������������
  //! \param tolerance [in] ��ֵ��
  //! \param pointIndexsToKeep [out] ��ֵ��
  //! \remarks
  //! ����ԭʼ�㼯����ֵ��ȡ�����������µĵ㼯�����pointIndexsToKeep
  void DouglasPeuckerReduction(std::vector<Point3d> points, int firstPoint,
                               int lastPoint, double tolerance,
                               std::list<int> &pointIndexsToKeep);

  //! \brief ������˹�տ˳�ϣͼ�κ�����
  //! \param pGeometry [in] Geometry����
  //! \param Tolerance [in] ��ֵ��
  //! \remarks ���ݸ����ļ��ζ������ֵ����һ��������ζ���
  Geometry *DouglasPeucker(Geometry *pGeometry, double Tolerance);

  virtual void OnChange();

 protected:
  virtual void SetBoundDirty(bool bDirty);

  virtual void Reset();

 protected:
  GeometryType m_geoType;

  std::vector<Point3d> m_pointSet;
  BoundBox3d m_boundBox;
  bool m_bDirty;
};

}  // namespace geditor
