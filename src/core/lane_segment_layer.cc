
#include "core/lane_segment_layer.h"

#include <time.h>

#include <algorithm>

#include "core/bound_segment.h"
#include "core/boundary_layer.h"
#include "core/camera.h"
#include "core/factory_drawable.h"
#include "core/geo_circular_arc.h"
#include "core/geo_rectangle.h"
#include "map/feature_type.h"
#include "map/projection_utm.h"
#include "map/vdb_manage.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_hdw_vertex_buffer.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_technique_manager.h"
#include "renderGL/mc_vertex_buffer_accessor.h"
#include "renderGL/mc_vertex_format.h"

namespace geditor
{

  SegmentLayer::SegmentLayer()
      : Layer(LT_LANE), line_width_(0.2), point_size_(0.1), check_(false) {}

  SegmentLayer::~SegmentLayer()
  {
    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      delete (*it);
    }
    lane_segment_.clear();
  }

  // 添加地图要素
  void SegmentLayer::AddMapFeature(MapFeature *feature)
  {
    if (feature != NULL)
    {
      if (feature->GetGeometry()->GetVertexCount() > 0)
      {
        feature->SetMapLayer(this);

        if (feature->GetUniqueID() == 0)
        {
          int uniqueId = GenerateFeatureID();
          feature->SetUniqueID(uniqueId);
        }

        lane_segment_.push_back((LaneSegment *)feature);
      }
    }
  }

  bool SegmentLayer::GetBoundary(V3d &vMin, V3d &vMax)
  {
    if (lane_segment_.size() > 0)
    {
      for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
           it != lane_segment_.end(); it++)
      {
        Geometry *pGeo = (*it)->GetGeometry();
        pGeo->CalculateBoundBox();

        const BoundBox3d &box = pGeo->GetBound();

        if (box.v_min_[0] < vMin[0])
          vMin[0] = box.v_min_[0];
        if (box.v_min_[1] < vMin[1])
          vMin[1] = box.v_min_[1];
        if (box.v_min_[2] < vMin[2])
          vMin[2] = box.v_min_[2];
        if (box.v_max_[0] > vMax[0])
          vMax[0] = box.v_max_[0];
        if (box.v_max_[1] > vMax[1])
          vMax[1] = box.v_max_[1];
        if (box.v_max_[2] > vMax[2])
          vMax[2] = box.v_max_[2];
      }
      return true;
    }

    return false;
  }

  // 拾取对象
  bool SegmentLayer::PickupObject(const Point3d &pnt3d, double tolerance,
                                  PickupResult &reslut)
  {
    bool bFind = false;

    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      Point3d outpnt;
      double fal;

      Geometry *pGeomLine = (*it)->GetGeometry();
      if (pGeomLine->GetGeometryType() == Geometry::GT_POLYLINE ||
          pGeomLine->GetGeometryType() == Geometry::GT_BEZIER_CURVE ||
          pGeomLine->GetGeometryType() == Geometry::GT_ARC_LINE ||
          pGeomLine->GetGeometryType() == Geometry::GT_BSPLINE_CURVE)
      {
        int index = pGeomLine->IsPointInEdge(pnt3d, outpnt, tolerance, fal);
        if (index > 0)
        {
          if (fal < reslut.dDistance)
          {
            reslut.pFeatureObject = (*it);

            reslut.ptNearPoint = outpnt;
            reslut.nSegmentIdx = index;
            reslut.dDistance = fal;

            bFind = true;
          }
        }
      }
      else if (pGeomLine->GetGeometryType() == Geometry::GT_HERMITE_CURVE)
      {
        LOG(ERROR) << "Hermite curve not supported yet";
        // Point3d outpnt;
        // GeoHermiteCurve3 *polyline = dynamic_cast<GeoHermiteCurve3
        // *>(pGeomLine); int index = polyline->IsPointInEdge(pnt3d, outpnt,
        // tolerance, fal); if (index > 0) {
        //     if (fal < reslut.dDistance) {
        //         reslut.pFeatureObject = (*it);

        //         reslut.ptNearPoint = outpnt;
        //         reslut.nSegmentIdx = index;
        //         reslut.dDistance = fal;

        //         bFind = true;
        //     }
        // }
      }
    }

    if (bFind)
    {
      int nIndex =
          OnPoint(reslut.pFeatureObject->GetGeometry(), pnt3d, tolerance);
      reslut.nKeyPoint = nIndex;
    }

    return bFind;
  }

  bool SegmentLayer::DeleteMapFeature(MapFeature *polyline)
  {
    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      if (polyline == (*it))
      {
        lane_segment_.erase(it);
        return true;
      }
    }
    return false;
  }

  int SegmentLayer::OnPoint(Geometry *geometry, const Point3d &Q,
                            double tolerance)
  {
    int index = -1;
    double minFlag = 100;

    int nSize = geometry->GetVertexCount();
    for (int i = 0; i < nSize; i++)
    {
      Point3d point = geometry->GetVertex(i);

      double maxVal =
          Mathd::Max(Mathd::Abs(point.x - Q.x), Mathd::Abs(point.y - Q.y));
      if (maxVal < minFlag)
      {
        minFlag = maxVal;
        index = i;
      }
    }

    if (minFlag < tolerance)
    {
      return index;
    }
    else
    {
      return -1;
    }
  }

  Geometry *CreateLine(int type)
  {
    Geometry *polyline = NULL;
    if (type == Geometry::GT_POLYLINE)
    {
      polyline = new GeoPolyline();
    }
    else if (type == Geometry::GT_CIRCULAR_ARC)
    {
      polyline = new GeoCircularArc();
    }
    else if (type == Geometry::GT_BEZIER_CURVE)
    {
      polyline = new GeoBezierCurve3();
    }
    else if (type == Geometry::GT_BSPLINE_CURVE)
    {
      polyline = new GeoBSplineCurve3();
    }
    else if (type == Geometry::GT_HERMITE_CURVE)
    {
      LOG(ERROR) << "Hermite curve not supported yet";
      // polyline = new GeoHermiteCurve3();
    }

    return polyline;
  }

  void SegmentLayer::BreakPolyline(MapFeature *pObject, int index,
                                   const Point3d &nearPnt3d)
  {
    if (pObject != NULL)
    {
      Geometry *pline = pObject->GetGeometry();

      Geometry *pNewLine = CreateLine(pline->GetGeometryType());
      pNewLine->AppendVertex(nearPnt3d);
      for (int i = index; i < pline->GetVertexCount(); i++)
      {
        const Point3d &pnt = pline->GetVertex(i);
        pNewLine->AppendVertex(pnt);
      }

      pline->Resize(index);
      pline->AppendVertex(nearPnt3d);

      LaneSegment *pLaneSegment = new LaneSegment();
      pLaneSegment->SetGeometry(pNewLine);

      this->AddMapFeature(pLaneSegment);
    }
  }

  bool Merge(std::vector<SegmentNode *> arrayNode, int segment,
             SegmentType type)
  {
    for (int i = 0; i < arrayNode.size(); i++)
    {
      if (arrayNode[i]->segment == segment && arrayNode[i]->type == type)
      {
        return true;
      }
    }

    return false;
  }

  void SegmentLayer::GenerateSegmentRelation(
      std::vector<LaneSegment *> &laneSegment, LaneSegment *lane)
  {
    // 建立道路关联关系
    int iCount = laneSegment.size();
    if (!lane)
    {
      for (int i = 0; i < iCount; i++)
      {
        laneSegment[i]->ClearSuccessorSegment();
        laneSegment[i]->ClearPredecessorSegment();
        laneSegment[i]->SetChanged(true);
      }
    }
    for (int i = 0; i < iCount - 1; i++)
    {
      if (lane && laneSegment[i] != lane)
        continue;
      for (int j = 1 + i; j < iCount; j++)
      {
        Geometry *pLine1 = laneSegment[i]->GetGeometry();
        Geometry *pLine2 = laneSegment[j]->GetGeometry();

        Point3d pt1Start = pLine1->GetStartVertex();
        Point3d pt1End = pLine1->GetEndVertex();

        Point3d pt2Start = pLine2->GetStartVertex();
        Point3d pt2End = pLine2->GetEndVertex();

        double dDist;

        dDist = Point3d::Distance(pt1End, pt2Start);
        if (dDist < point_size_ * 3)
        {
          laneSegment[i]->AddSuccessorSegment(laneSegment[j]);
          laneSegment[j]->AddPredecessorSegment(laneSegment[i]);
        }

        dDist = Point3d::Distance(pt2End, pt1Start);
        if (dDist < point_size_ * 3)
        {
          laneSegment[j]->AddSuccessorSegment(laneSegment[i]);
          laneSegment[i]->AddPredecessorSegment(laneSegment[j]);
        }
      }
      laneSegment[i]->SetChanged(true);
    }
  }

  MapFeature *SegmentLayer::GetFeature(int idx)
  {
    return GetFeatureVec(idx, lane_segment_);
  }

  int SegmentLayer::GetAllSegment(std::vector<LaneSegment *> &laneSegment)
  {
    int iCount = lane_segment_.size();
    for (int i = 0; i < iCount; i++)
    {
      laneSegment.push_back(lane_segment_[i]);
    }
    return iCount;
  }

  LaneSegment *SegmentLayer::GetAllParallelSegment(
      LaneSegment *pSegment, std::vector<LaneSegment *> &laneSegment)
  {
    ParallelSegment *pParallel = pSegment->GetParallelSegment();

    LaneSegment *pLeftLane = NULL;
    {
      LaneSegment *pLeftSegment = NULL;
      int nleftSegment = pParallel->leftSegment;
      do
      {
        pLeftSegment = (LaneSegment *)GetFeature(nleftSegment);
        if (pLeftSegment != NULL)
        {
          pLeftLane = pLeftSegment;
          laneSegment.push_back(pLeftSegment);

          pParallel = pLeftSegment->GetParallelSegment();
          nleftSegment = pParallel->leftSegment;
        }

      } while (pLeftSegment != NULL);

      if (laneSegment.size() > 0)
      {
        pLeftSegment = laneSegment.back();
      }
    }

    LaneSegment *pRightLane = NULL;
    {
      LaneSegment *pRightSegment = NULL;
      int nRightSegment = pParallel->rightSegment;
      do
      {
        pRightSegment = (LaneSegment *)GetFeature(nRightSegment);
        if (pRightSegment != NULL)
        {
          pRightLane = pRightSegment;
          laneSegment.push_back(pRightSegment);

          pParallel = pRightSegment->GetParallelSegment();
          nRightSegment = pParallel->rightSegment;
        }

      } while (pRightSegment != NULL);
    }

    if (pLeftLane != NULL)
    {
      return pLeftLane;
    }
    else
    {
      return pSegment;
    }
  }

  void SegmentLayer::GenerateRoadRelation(
      std::vector<LaneSegment *> &laneSegment)
  {
    // 生成道路ID
    int iCount = laneSegment.size();
    for (int i = 0; i < iCount; i++)
    {
      // 获取所有平行道路
      std::vector<LaneSegment *> parallelSegment;
      LaneSegment *pLeftLane =
          GetAllParallelSegment(laneSegment[i], parallelSegment);

      // 设置当前车道
      RoadSegment *pLinkRoad = laneSegment[i]->GetParentLink();
      if (!pLinkRoad)
      {
        pLinkRoad = new RoadSegment();
        pLinkRoad->SetUniqueID(pLeftLane->GetUniqueID());

        laneSegment[i]->SetParentLink(pLinkRoad);
        pLinkRoad->AddLaneSegment(laneSegment[i]);

        road_link_.push_back(pLinkRoad);
      }

      // 关联所有平行车道
      for (int j = 0; j < parallelSegment.size(); j++)
      {
        RoadSegment *pRoad = parallelSegment[j]->GetParentLink();
        if (!pRoad)
        {
          parallelSegment[j]->SetParentLink(pLinkRoad);
          pLinkRoad->AddLaneSegment(parallelSegment[j]);
        }
        else
        {
          parallelSegment[j]->SetParentLink(pLinkRoad);
          pLinkRoad->AddLaneSegment(parallelSegment[j]);
        }
      }
    }

    // 生成拓扑
    for (int i = 0; i < road_link_.size(); i++)
    {
      std::vector<LaneSegment *> arraySeg;
      road_link_[i]->GetLaneSegment(arraySeg);

      for (int j = 0; j < arraySeg.size(); j++)
      {
        std::vector<LaneSegment *> segSuccessor;
        arraySeg[j]->GetSuccessorSegment(segSuccessor);
        for (int x = 0; x < segSuccessor.size(); x++)
        {
          RoadSegment *pLinkRoad = segSuccessor[x]->GetParentLink();
          road_link_[i]->AddSuccessorRoad(pLinkRoad);
        }

        std::vector<LaneSegment *> segPredecessor;
        arraySeg[j]->GetPredecessorSegment(segPredecessor);
        for (int x = 0; x < segPredecessor.size(); x++)
        {
          RoadSegment *pLinkRoad = segPredecessor[x]->GetParentLink();
          road_link_[i]->AddPredecessorRoad(pLinkRoad);
        }
      }
    }
  }

  void SegmentLayer::GenerateSegmentNodeID(
      const std::vector<LaneSegment *> &laneSegment,
      std::map<int, std::vector<SegmentNode *>> &segmentNode)
  {
    struct NodePoint
    {
      int nodeId;
      LaneSegment *pEnter;
      LaneSegment *pExit;
      Point3d point;
    };

    std::vector<NodePoint> nodeArray;

    // 建立道路关联关系
    int iCount = laneSegment.size();
    for (int i = 0; i < iCount - 1; i++)
    {
      for (int j = 1 + i; j < iCount; j++)
      {
        Geometry *pLine1 = laneSegment[i]->GetGeometry();
        Geometry *pLine2 = laneSegment[j]->GetGeometry();

        if (laneSegment[i]->IsSuccessorSegment(laneSegment[j]))
        {
          NodePoint node;

          Point3d ptEnd = laneSegment[i]->GetGeometry()->GetEndVertex();
          Point3d ptStart = laneSegment[j]->GetGeometry()->GetStartVertex();

          node.point = Point3d::MiddlePoint(ptEnd, ptStart);
          node.pEnter = laneSegment[i];
          node.pExit = laneSegment[j];
          node.nodeId = 0;

          nodeArray.push_back(node);
        }

        if (laneSegment[j]->IsSuccessorSegment(laneSegment[i]))
        {
          NodePoint node;

          Point3d ptEnd = laneSegment[j]->GetGeometry()->GetEndVertex();
          Point3d ptStart = laneSegment[i]->GetGeometry()->GetStartVertex();

          node.point = Point3d::MiddlePoint(ptEnd, ptStart);
          node.pEnter = laneSegment[j];
          node.pExit = laneSegment[i];
          node.nodeId = 0;

          nodeArray.push_back(node);
        }
      }
    }

    // 生成节点ID，并进行节点合并
    int nNodeSeq = GenerateFeatureID();
    int iSzie = nodeArray.size();
    for (int i = 0; i < iSzie; i++)
    {
      if (nodeArray[i].nodeId != 0)
      {
        continue;
      }

      nodeArray[i].nodeId = nNodeSeq;

      Point3d point = nodeArray[i].point;
      for (int j = i + 1; j < iSzie; j++)
      {
        if (nodeArray[j].nodeId == 0)
        {
          double dDist = Point3d::Distance(nodeArray[j].point, point);
          if (dDist < point_size_ * 3)
          {
            nodeArray[j].nodeId = nNodeSeq;
          }
        }
      }
      nNodeSeq++;
    }

    // 生成节点关系，并去重。
    int nSzie = nodeArray.size();
    for (int i = 0; i < nSzie; i++)
    {
      std::map<int, std::vector<SegmentNode *>>::iterator it_find =
          segmentNode.find(nodeArray[i].nodeId);

      if (it_find != segmentNode.end())
      {
        if (!Merge(it_find->second, nodeArray[i].pEnter->GetUniqueID(),
                   ENTER_SEGMENT))
        {
          SegmentNode *pEnterSegment = new SegmentNode();
          pEnterSegment->segment = nodeArray[i].pEnter->GetUniqueID();
          pEnterSegment->type = ENTER_SEGMENT;

          it_find->second.push_back(pEnterSegment);
        }

        if (!Merge(it_find->second, nodeArray[i].pExit->GetUniqueID(),
                   EXIT_SEGMENT))
        {
          SegmentNode *pEixtSegment = new SegmentNode();
          pEixtSegment->segment = nodeArray[i].pExit->GetUniqueID();
          pEixtSegment->type = EXIT_SEGMENT;

          it_find->second.push_back(pEixtSegment);
        }
      }
      else
      {
        std::vector<SegmentNode *> arraySegment;

        SegmentNode *pEnterSegment = new SegmentNode();
        pEnterSegment->segment = nodeArray[i].pEnter->GetUniqueID();
        pEnterSegment->type = ENTER_SEGMENT;

        arraySegment.push_back(pEnterSegment);

        SegmentNode *pEixtSegment = new SegmentNode();
        pEixtSegment->segment = nodeArray[i].pExit->GetUniqueID();
        pEixtSegment->type = EXIT_SEGMENT;

        arraySegment.push_back(pEixtSegment);

        segmentNode.insert(std::pair<int, std::vector<SegmentNode *>>(
            nodeArray[i].nodeId, arraySegment));
      }
    }
  }

  bool InPolygon(Geometry *pGeometry, const Point3d &p)
  {
    pGeometry->CalculateBoundBox();
    BoundBox3d bound = pGeometry->GetBound();

    if (!bound.Contains(V3d(p.x, p.y, p.z)))
    {
      return false;
    }

    bool inside = false;

    int mNumPoints = pGeometry->GetVertexCount();
    for (int i = 0, j = mNumPoints - 1; i < mNumPoints; j = i++)
    {
      Point3d const &U0 = pGeometry->GetVertex(i);
      Point3d const &U1 = pGeometry->GetVertex(j);

      double rhs, lhs;

      if (p.y < U1.y)
      {
        if (U0.y <= p.y)
        {
          lhs = (p.y - U0.y) * (U1.x - U0.x);
          rhs = (p.x - U0.x) * (U1.y - U0.y);
          if (lhs > rhs)
          {
            inside = !inside;
          }
        }
      }
      else if (p.y < U0.y) // U1 on or below ray, U0 above ray
      {
        lhs = (p.y - U0.y) * (U1.x - U0.x);
        rhs = (p.x - U0.x) * (U1.y - U0.y);
        if (lhs < rhs)
        {
          inside = !inside;
        }
      }
    }
    return inside;
  }

  void SegmentLayer::GenerateTopology()
  {
    GenerateLaneTopology();
    std::vector<JobArea *> jobs;
    auto joblayer = (JobLayer *)layers_.FunareaLayer();
    if (joblayer)
      joblayer->GetAllJobArea(jobs);
    GenerateJobTopology(jobs);
  }

  void SegmentLayer::GenerateJobTopology(std::vector<JobArea *> &jobs)
  {
    std::vector<LaneSegment *> laneSegment = lane_segment_;
    std::vector<JobArea *> joba, jobp;

    for (int j = 0; j < jobs.size(); j++)
    {
      jobs[j]->ClearAttachObject();
      if (jobs[j]->GetProperty()->areaType < TV::jobpoint)
        joba.push_back(jobs[j]);
      else
      {
        jobp.push_back(jobs[j]);
      }
    }
    for (int i = 0; i < laneSegment.size(); i++)
    {
      laneSegment[i]->SetPredecessorFeature(nullptr);
      laneSegment[i]->SetSuccessorFeature(nullptr);
      Geometry *geomtry = laneSegment[i]->GetGeometry();
      Point3d sPnt = geomtry->GetStartVertex();
      Point3d ePnt = geomtry->GetEndVertex();

      for (int j = 0; j < joba.size(); j++)
      {
        Geometry *geoPolygon = joba[j]->GetGeometry();

        bool bSInside = InPolygon(geoPolygon, sPnt);
        bool bEInside = InPolygon(geoPolygon, ePnt);

        if (bSInside && bEInside)
        {
          laneSegment[i]->SetPredecessorFeature(joba[j]);
          laneSegment[i]->SetSuccessorFeature(joba[j]);
        }
        else if (bSInside)
        {
          laneSegment[i]->SetPredecessorFeature(joba[j]);
        }
        else if (bEInside)
        {
          laneSegment[i]->SetSuccessorFeature(joba[j]);
        }
      }
    }
    for (int i = 0; i < jobp.size(); i++)
    {
      auto sp = jobp[i]->GetGeometry()->GetStartVertexPtr();
      auto ep = jobp[i]->GetGeometry()->GetEndVertexPtr();
      for (int j = 0; j < joba.size(); j++)
      {
        Geometry *geoPolygon = joba[j]->GetGeometry();
        bool bSinside = InPolygon(geoPolygon, *sp);
        bool bEInside = InPolygon(geoPolygon, *ep);
        if (bSinside || bEInside)
          joba[j]->AddAttachObject(jobp[i]);
      }
    }
  }
  void SegmentLayer::GenerateLaneTopology(LaneSegment *lane)
  {
    if (lane)
    {
      GenerateSegmentRelation(lane_segment_, lane);
    }
    else
    {
      GenerateSegmentRelation(lane_segment_);
      OverlapLanesVertex();
      GenerateRoadRelation(lane_segment_);
    }
  }
  void SegmentLayer::OverlapLanesVertex()
  {
    int iCount = lane_segment_.size();
    for (int i = 0; i < iCount; i++)
    {
      lane_segment_[i]->OverlapLaneVertex(1);
    }
    for (int i = 0; i < iCount; i++)
    {
      lane_segment_[i]->OverlapLaneVertex(0);
    }
  }
  void SegmentLayer::SetParallelSegment(
      std::vector<LaneSegment *> selectSegment)
  {
    std::vector<int> sequence;

    int iCount = selectSegment.size();
    for (int i = 0; i < iCount; i++)
    {
      sequence.push_back(i);
      selectSegment[i]->SetLeftSegment(nullptr);
      selectSegment[i]->SetRightSegment(nullptr);
    }

    for (int i = 0; i < iCount - 1; i++)
    {
      for (int j = i + 1; j < iCount; j++)
      {
        GeoPolyline *pLine1 =
            dynamic_cast<GeoPolyline *>(selectSegment[i]->GetGeometry());
        GeoPolyline *pLine2 =
            dynamic_cast<GeoPolyline *>(selectSegment[j]->GetGeometry());

        int positive = 0;
        int negative = 0;

        for (int x = 0; x < pLine1->GetVertexCount(); x++)
        {
          Point3d point3d = pLine1->GetVertex(x);
          double nearPt = pLine2->GetNeartPoint(point3d);
          if (nearPt > 0.0)
          {
            positive++;
          }
          else if (nearPt < 0.0)
          {
            negative++;
          }
        }

        if (positive < negative)
        {
          int temp = sequence[i];
          sequence[i] = sequence[j];
          sequence[j] = temp;
        }
      }
    }

    for (int i = 0; i < iCount; i++)
    {
      std::vector<int>::iterator iter;

      iter = std::find(sequence.begin(), sequence.end(), sequence[i] - 1);
      if (iter != sequence.end())
      {
        int index = std::distance(std::begin(sequence), iter);
        selectSegment[i]->SetLeftSegment(selectSegment[index]);
      }

      iter = std::find(sequence.begin(), sequence.end(), sequence[i] + 1);
      if (iter != sequence.end())
      {
        int index = std::distance(std::begin(sequence), iter);
        selectSegment[i]->SetRightSegment(selectSegment[index]);
      }
      selectSegment[i]->SetChanged(true);
    }
  }

  void SegmentLayer::SetReverseSegment(std::vector<LaneSegment *> selectSegment)
  {
    selectSegment[0]->SetLeftReverseSegment(selectSegment[1]);
    selectSegment[0]->SetRightReverseSegment(nullptr);
    selectSegment[0]->SetChanged(true);
    selectSegment[1]->SetLeftReverseSegment(selectSegment[0]);
    selectSegment[1]->SetRightReverseSegment(nullptr);
    selectSegment[1]->SetChanged(true);
  }

  //
  void SegmentLayer::AddMapObject(std::vector<Lane *> segmentArray)
  {
    int featureId = GenerateFeatureID();

    for (int i = 0; i < segmentArray.size(); i++)
    {
      LaneSegment *pLaneSegment = new LaneSegment();

      Geometry *pPolyline = segmentArray[i]->polyline;
      if (pPolyline->GetGeometryType() == Geometry::GT_HERMITE_CURVE)
      {
        LOG(ERROR) << "Hermite curve not supported yet";
        // ((GeoHermiteCurve3 *) pPolyline)->Update();
      }

      if (segmentArray[i]->uniqueId == 0)
      {
        pLaneSegment->SetUniqueID(featureId + i);
      }
      else
      {
        pLaneSegment->SetUniqueID(segmentArray[i]->uniqueId);
      }

      pLaneSegment->SetGeometry(segmentArray[i]->polyline);
      pLaneSegment->SetProperty(&segmentArray[i]->pProperty);

      AddMapFeature(pLaneSegment);
      delete segmentArray[i];
    }
  }
  void SegmentLayer::ClearLayer()
  {
    for (int i = 0; i < lane_segment_.size(); i++)
    {
      delete lane_segment_[i];
    }
    lane_segment_.clear();

    for (std::vector<RoadSegment *>::iterator iter = road_link_.begin();
         iter != road_link_.end(); iter++)
    {
      delete (*iter);
    }
    road_link_.clear();
  }

  RoadSegment *SegmentLayer::FindRoadSegment(int linkId)
  {
    for (int i = 0; i < road_link_.size(); i++)
    {
      if (road_link_[i]->GetUniqueID() == linkId)
      {
        return road_link_[i];
      }
    }

    return NULL;
  }

  void SegmentLayer::Read(VDBManage *vdb)
  {
    if (vdb != NULL)
    {
      // 先清空图层
      ClearLayer();

      std::vector<Lane *> segmentArray;
      if (vdb->ReadSegmentProperty(segmentArray))
      {
        AddMapObject(segmentArray);
      }
      auto *boundary_layer = (BoundaryLayer *)layers_.BoundaryLayer();
      for (int i = 0; i < lane_segment_.size(); i++)
      {
        ParallelSegment parallel;

        int uniqueId = lane_segment_[i]->GetUniqueID();

        // 读取平行关系
        if (vdb->ReadParallelSegment(uniqueId, &parallel))
        {
          auto *l = GetFeature(parallel.leftSegment);
          auto *r = GetFeature(parallel.rightSegment);
          lane_segment_[i]->SetLeftSegment(l);
          lane_segment_[i]->SetRightSegment(r);
          auto *l_r = GetFeature(parallel.leftReverseSegment);
          auto *r_r = GetFeature(parallel.rightReverseSegment);
          lane_segment_[i]->SetLeftReverseSegment(l_r);
          lane_segment_[i]->SetRightReverseSegment(r_r);
        }

        // 读取link segment关系
        //  int linkId;
        //  if (vdb->ReadLinkSegmentRelation(uniqueId, &linkId)) {
        //    /*
        //    RoadSegment * pLinkRoad = FindRoadSegment(linkId);
        //    if (!pLinkRoad)
        //    {
        //    RoadSegment * pLinkRoad = new RoadSegment();
        //    pLinkRoad->SetUniqueID(linkId);

        //   m_RoadLink.push_back(pLinkRoad);
        //   }

        //   m_LaneSegment[i]->SetParentLink(pLinkRoad);
        //   pLinkRoad->AddLaneSegment(m_LaneSegment[i]);
        //   */
        // }

        // int jobPredecessor;
        // int jobSuccessor;
        // if (vdb->ReadJobSegmentRelation(uniqueId, jobPredecessor,
        // jobSuccessor)) {
        //   auto jobp = layers_.FunareaLayer()->GetFeature(jobPredecessor);
        //   if (jobp) lane_segment_[i]->SetPredecessorFeature(jobp);
        //   auto jobs = layers_.FunareaLayer()->GetFeature(jobSuccessor);
        //   if (jobs) lane_segment_[i]->SetSuccessorFeature(jobs);
        // }
        int lid = lane_segment_[i]->GetProperty()->leftBoundary;
        if (lid > 0 && boundary_layer)
        {
          auto *seg = boundary_layer->GetFeature(lid);
          if (seg)
            lane_segment_[i]->SetLeftBoundary(seg);
        }
        int rid = lane_segment_[i]->GetProperty()->rightBoundary;
        if (rid > 0 && boundary_layer)
        {
          auto *seg = boundary_layer->GetFeature(rid);
          if (seg)
            lane_segment_[i]->SetRightBoundary(seg);
        }

        std::vector<std::pair<int, int>> objs;
        vdb->ReadAttachObject(uniqueId, objs);
        for (auto &obj : objs)
        {
          auto *feat =
              layers_.GetLayerByFeature(obj.second)->GetFeature(obj.first);
          if (feat)
            lane_segment_[i]->AddAttachObject(feat);
        }
      }
      // std::vector<SegmentNode *> snodes;
      // std::vector<LaneSegment *> seg_enter, seg_exit;
      // if (vdb->ReadSegNode(snodes)) {
      //   for (auto &n : snodes) {
      //     auto seg = (LaneSegment *)GetFeature(n->segment);
      //     if (!seg) continue;
      //     if (n->type == ENTER_SEGMENT)
      //       seg_enter.push_back(seg);
      //     else if (n->type == EXIT_SEGMENT)
      //       seg_exit.push_back(seg);
      //   }
      //   for (auto &s : seg_enter)
      //     for (auto &st : seg_exit) {
      //       s->SetSuccessorFeature(st);
      //       st->SetPredecessorFeature(s);
      //     }
      // }
    }
  }

  Geometry *SegmentLayer::MergeGeometry(Geometry *pLine1, Geometry *pLine2)
  {
    if (pLine1 != NULL && pLine2 != NULL)
    {
      Point3d p1SV = pLine1->GetStartVertex();
      Point3d p1EV = pLine1->GetEndVertex();

      Point3d p2SV = pLine2->GetStartVertex();
      Point3d p2EV = pLine2->GetEndVertex();

      double dDist;
      dDist = Point3d::Distance(p1SV, p2SV);
      if (dDist < point_size_ * 3)
      {
        pLine1->ReverseVertex();

        int iCount = pLine2->GetVertexCount();
        for (int i = 0; i < iCount; i++)
        {
          const Point3d &point = pLine2->GetVertex(i);
          pLine1->AppendVertex(point);
        }
        return pLine1;
      }

      dDist = Point3d::Distance(p1EV, p2EV);
      if (dDist < point_size_ * 3)
      {
        int iCount = pLine2->GetVertexCount();
        for (int i = iCount - 1; i >= 0; i--)
        {
          const Point3d &point = pLine2->GetVertex(i);
          pLine1->AppendVertex(point);
        }
        return pLine1;
      }

      dDist = Point3d::Distance(p1SV, p2EV);
      if (dDist < point_size_ * 3)
      {
        pLine1->ReverseVertex();

        int iCount = pLine2->GetVertexCount();
        for (int i = iCount - 1; i >= 0; i--)
        {
          const Point3d &point = pLine2->GetVertex(i);
          pLine1->AppendVertex(point);
        }
        return pLine1;
      }

      dDist = Point3d::Distance(p1EV, p2SV);
      if (dDist < point_size_ * 3)
      {
        int iCount = pLine2->GetVertexCount();
        for (int i = 0; i < iCount; i++)
        {
          const Point3d &point = pLine2->GetVertex(i);
          pLine1->AppendVertex(point);
        }

        return pLine1;
      }
    }
    return NULL;
  }

  void SegmentLayer::MergeObject(std::vector<LaneSegment *> selectSegment)
  {
    int iCount = selectSegment.size();
    for (int i = 0; i < iCount - 1; i++)
    {
      for (int j = 1 + i; j < iCount; j++)
      {
        Geometry *pLine1 = selectSegment[i]->GetGeometry();
        Geometry *pLine2 = selectSegment[j]->GetGeometry();

        Geometry *pMergeLine = MergeGeometry(pLine1, pLine2);
        if (pMergeLine != NULL)
        {
          DeleteMapFeature(selectSegment[j]);
          return;
        }
      }
    }
  }

  void SegmentLayer::GenerateRoadNodeID(
      const std::vector<RoadSegment *> &linkSegment,
      std::map<int, std::vector<SegmentNode *>> &segmentNode)
  {
    for (int i = 0; i < linkSegment.size(); i++)
    {
      std::vector<SegmentNode *> nodeArray;

      SegmentNode *pNode = new SegmentNode();
      pNode->segment = linkSegment[i]->GetUniqueID();
      pNode->type = EXIT_SEGMENT;

      nodeArray.push_back(pNode);

      //===============================

      std::vector<RoadSegment *> linkPredecessor;
      linkSegment[i]->GetPredecessorRoad(linkPredecessor);
      if (linkPredecessor.size() <= 0)
      {
        continue;
      }

      for (int x = 0; x < linkPredecessor.size(); x++)
      {
        SegmentNode *pNode = new SegmentNode();
        pNode->segment = linkPredecessor[x]->GetUniqueID();
        pNode->type = ENTER_SEGMENT;

        nodeArray.push_back(pNode);

        std::vector<RoadSegment *> segArray;
        linkPredecessor[x]->GetSuccessorRoad(segArray);
        for (int y = 0; y < segArray.size(); y++)
        {
          if (segArray[y] != linkSegment[i])
          {
            SegmentNode *pNode = new SegmentNode();
            pNode->segment = segArray[y]->GetUniqueID();
            pNode->type = EXIT_SEGMENT;

            nodeArray.push_back(pNode);
          }
        }
      }
      segmentNode.insert(
          std::pair<int, std::vector<SegmentNode *>>(i + 1, nodeArray));
    }
  }
  void SegmentLayer::CheckDeletedRealtion()
  {
    for (std::vector<LaneSegment *>::iterator iter = lane_segment_.begin();
         iter != lane_segment_.end(); iter++)
    {
      (*iter)->CheckDeletedRealtion();
    }
  }
  void SegmentLayer::Save(VDBManage *vdb)
  {
    if (vdb != NULL)
    {
      vdb->ClearSegmentProperty();
      vdb->ClearParallelSegment();
      vdb->ClearLinkSegmentRelation();
      CheckDeletedRealtion();
      for (std::vector<LaneSegment *>::iterator iter = lane_segment_.begin();
           iter != lane_segment_.end(); iter++)
      {
        LaneSegment *pLaneSegment = (*iter);
        if (pLaneSegment->deleted())
          continue;
        char *pMem = NULL;
        int length = PackGeometry(pLaneSegment->GetGeometry(), pMem);

        // 保存车道属性信息
        vdb->SaveSegmentProperty(pLaneSegment->GetUniqueID(),
                                 pLaneSegment->GetProperty(), pMem, length);

        // 保存平行道路信息

        vdb->SaveParallelSegment(pLaneSegment->GetUniqueID(),
                                 pLaneSegment->GetParallelSegment());

        int nPreFeature = 0;
        MapFeature *pPreFeature = pLaneSegment->GetPredecessorFeature();
        if (pPreFeature != NULL)
        {
          nPreFeature = pPreFeature->GetUniqueID();
        }

        // 保存job信息
        int nSucFeature = 0;
        MapFeature *pSucFeature = pLaneSegment->GetSuccessorFeature();
        if (pSucFeature != NULL)
        {
          nSucFeature = pSucFeature->GetUniqueID();
        }
        vdb->SaveJobSegmentRelation(pLaneSegment->GetUniqueID(), nPreFeature,
                                    nSucFeature);

        // 保存车道附属信息
        std::vector<MapFeature *> attachObjects;
        pLaneSegment->GetAttachObject(attachObjects);
        if (attachObjects.size() > 0)
        {
          vdb->SaveAttachObject(pLaneSegment->GetUniqueID(), attachObjects);
        }

        // 保存道路与车道关系信息
        RoadSegment *pLinkRoad = pLaneSegment->GetParentLink();
        if (pLinkRoad)
        {
          vdb->SaveLinkSegmentRelation(pLaneSegment->GetUniqueID(),
                                       pLinkRoad->GetUniqueID());
        }
        else
        {
          vdb->SaveLinkSegmentRelation(pLaneSegment->GetUniqueID(), 0);
        }
      }

      // 道路属性
      vdb->ClearRoadLink();
      for (std::vector<RoadSegment *>::iterator iter = road_link_.begin();
           iter != road_link_.end(); iter++)
      {
        vdb->SaveRoadLink((*iter)->GetUniqueID());
      }

      // 道路拓扑
      std::map<int, std::vector<SegmentNode *>> roadNode;
      GenerateRoadNodeID(road_link_, roadNode);

      vdb->ClearRoadNodeRelation();
      for (std::map<int, std::vector<SegmentNode *>>::iterator iter =
               roadNode.begin();
           iter != roadNode.end(); iter++)
      {
        std::vector<SegmentNode *> nodeArray = iter->second;
        int nodeId = iter->first;
        for (int i = 0; i < nodeArray.size(); i++)
        {
          vdb->SaveRoadNodeRelation(nodeId, nodeArray[i]);
        }
      }

      // 车道拓扑
      std::map<int, std::vector<SegmentNode *>> segmentNode;
      GenerateSegmentNodeID(lane_segment_, segmentNode);

      vdb->ClearSegmentNodeRelation();
      for (std::map<int, std::vector<SegmentNode *>>::iterator iter =
               segmentNode.begin();
           iter != segmentNode.end(); iter++)
      {
        std::vector<SegmentNode *> nodeArray = iter->second;
        int nodeId = iter->first;
        for (int i = 0; i < nodeArray.size(); i++)
        {
          vdb->SaveSegmentNodeRelation(nodeId, nodeArray[i]);
        }
      }
    }
  }

  void SegmentLayer::SetPointSize(double x)
  {
    point_size_ = x;

    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      (*it)->SetChanged(true);
    }
  }

  void SegmentLayer::SetLineWidth(double x)
  {
    line_width_ = x;

    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      (*it)->SetChanged(true);
    }
  }

  void SegmentLayer::SetCarBodyCheck(bool bCheck)
  {
    check_ = bCheck;

    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      (*it)->SetChanged(true);
    }
  }

  void SegmentLayer::SetVizRoadRight(bool on)
  {
    viz_road_right_ = on;
    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      (*it)->SetChanged(true);
    }
  }

  void SegmentLayer::SetShowDirectionOverlay(bool on)
  {
    show_direction_overlay_ = on;
    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      (*it)->SetChanged(true);
    }
  }

  // 几何对象转换成渲染对象
  void SegmentLayer::Cull(double minX, double minY, double maxX, double maxY)
  {
    for (std::map<Geometry *, PositionTransformNode *>::iterator iter =
             render_leaf_.begin();
         iter != render_leaf_.end();)
    {
      bool bFind = false;

      for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
           it != lane_segment_.end(); it++)
      {
        Geometry *pPolyline = (*it)->GetGeometry();
        if (pPolyline == iter->first)
        {
          bFind = true;
          break;
        }
      }

      if (!bFind)
      {
        std::map<Geometry *, PositionTransformNode *>::iterator it_delete =
            iter++;
        render_leaf_.erase(it_delete);
      }
      else
      {
        iter++;
      }
    }

    for (std::vector<LaneSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++)
    {
      LaneSegment *pLaneSegment = (*it);

      Geometry *pPolyline = (*it)->GetGeometry();
      std::map<Geometry *, PositionTransformNode *>::iterator it_transformNode =
          render_leaf_.find(pPolyline);

      PositionTransformNode *pTransformNode = NULL;

      if (render_leaf_.end() == it_transformNode)
      {
        pTransformNode = new PositionTransformNode();
        render_leaf_.insert(std::pair<Geometry *, PositionTransformNode *>(
            pPolyline, pTransformNode));
      }
      else
      {
        if (pLaneSegment->IsChanged())
        {
          pTransformNode = it_transformNode->second;
          pLaneSegment->SetChanged(false);
        }
      }
      pLaneSegment->CheckDeletedRealtion();
      if (pTransformNode != NULL)
      {
        pTransformNode->RemoveAllChild();

        pPolyline->CalculateBoundBox();
        BoundBox3d bound = pPolyline->GetBound();
        V3d vCnt = bound.GetCenter();
        pTransformNode->SetPosition(vCnt);

        PolyLineSytle *pStyle = pLaneSegment->GetStyle();
        int highlightPoint = pLaneSegment->GetHighlightPoint();

        // 方向高亮激活 = 开启"设定行驶方向" 且本车道 direction !=0
        bool _dirActiveEarly =
            show_direction_overlay_ && pLaneSegment->GetProperty() &&
            (pLaneSegment->GetProperty()->direction == 1 ||
             pLaneSegment->GetProperty()->direction == 2);
        if (line_width_ > 0.01 && (*it)->InLaneNet() > 0 && !_dirActiveEarly)
        {
          double line_width = line_width_;
          if (check_)
          {
            std::vector<JobArea *> jobArea;
            int count =
                ((JobLayer *)layers_.FunareaLayer())->GetAllJobArea(jobArea);
            Drawable *pWidthLine = FactoryDrawable::CreateCarBodyLineDrawable(
                pPolyline, vCnt, line_width, pStyle->GetBackgroundColor(),
                jobArea);
            if (pWidthLine != NULL)
            {
              RenderTechnique *pTechnique =
                  TechniqueManager::GetInstance()->GetTechnique(1);

              RenderLeaf *pRenderLeaf = new RenderLeaf();
              pRenderLeaf->SetDrawable(pWidthLine);
              pRenderLeaf->SetRenderTechnique(pTechnique);

              pTransformNode->AddChild(pRenderLeaf);
            }
          }
          else
          {
            Drawable *pWidthLine = FactoryDrawable::CreateWidthLineDrawable(
                pPolyline, vCnt, line_width, pStyle->GetBackgroundColor());
            if (pWidthLine != NULL)
            {
              RenderTechnique *pTechnique =
                  TechniqueManager::GetInstance()->GetTechnique(3);

              RenderLeaf *pRenderLeaf = new RenderLeaf();
              pRenderLeaf->SetDrawable(pWidthLine);
              pRenderLeaf->SetRenderTechnique(pTechnique);

              pTransformNode->AddChild(pRenderLeaf);
            }
          }
        }
        int r =
            (pLaneSegment->selected() || pLaneSegment->highlighted()) ? 175 : 100;
        V4f col(1, 1, 1, 0.3);
        if (pLaneSegment->selected() || pLaneSegment->highlighted())
          col = V4f(1, 0, 0, 0.2);
        // road_right 可视化：road_right=1 的车道两侧填充改为浅绿
        // 但方向高亮开启且本车道已设置方向时，让方向色带独占，不再染绿
        if (viz_road_right_ && pLaneSegment->GetProperty() &&
            pLaneSegment->GetProperty()->road_right == 1 &&
            !pLaneSegment->selected() && !pLaneSegment->highlighted() &&
            !_dirActiveEarly)
        {
          col = V4f(0.2f, 0.8f, 0.30f, 0.6f);
        }
        std::vector<Point3d> itemslane;
        pPolyline->Hermite(itemslane);
        GeoPolygon polygonl;
        if ((*it)->GetLeftBoundary() && !(*it)->GetLeftBoundary()->deleted())
        {
          std::vector<Point3d> items;
          (*it)->GetLeftBoundary()->GetGeometry()->Hermite(items);
          for (size_t i = 0; i < items.size(); ++i)
          {
            polygonl.AppendVertex(items[i]);
          }
          for (size_t i = 0; i < itemslane.size(); ++i)
          {
            polygonl.AppendVertex(itemslane[itemslane.size() - i - 1]);
          }
          // polygonl.AppendVertex(pPolyline->GetEndVertex());
          // polygonl.AppendVertex(pPolyline->GetStartVertex());
        }
        Drawable *pArrow =
            FactoryDrawable::CreateAreaDrawable(&polygonl, vCnt, col);
        if (pArrow != NULL)
        {
          RenderTechnique *pTechnique =
              TechniqueManager::GetInstance()->GetTechnique(1);

          RenderLeaf *pRenderLeaf = new RenderLeaf();
          pRenderLeaf->SetDrawable(pArrow);
          pRenderLeaf->SetRenderTechnique(pTechnique);

          pTransformNode->AddChild(pRenderLeaf);
        }
        GeoPolygon polygonr;
        if ((*it)->GetRightBoundary() && !(*it)->GetRightBoundary()->deleted())
        {
          std::vector<Point3d> items;
          (*it)->GetRightBoundary()->GetGeometry()->Hermite(items);
          for (size_t i = 0; i < items.size(); ++i)
          {
            polygonr.AppendVertex(items[i]);
          }
          for (size_t i = 0; i < itemslane.size(); ++i)
          {
            polygonr.AppendVertex(itemslane[itemslane.size() - i - 1]);
          }
          // polygonr.AppendVertex(pPolyline->GetEndVertex());
          // polygonr.AppendVertex(pPolyline->GetStartVertex());
        }
        Drawable *pArrowr =
            FactoryDrawable::CreateAreaDrawable(&polygonr, vCnt, col);
        if (pArrowr != NULL)
        {
          RenderTechnique *pTechnique =
              TechniqueManager::GetInstance()->GetTechnique(1);

          RenderLeaf *pRenderLeaf = new RenderLeaf();
          pRenderLeaf->SetDrawable(pArrowr);
          pRenderLeaf->SetRenderTechnique(pTechnique);

          pTransformNode->AddChild(pRenderLeaf);
        }

        Drawable *pLine = FactoryDrawable::CreateLineDrawable(
            pPolyline, vCnt, pStyle->GetLineColor());
        if (pLine != NULL)
        {
          RenderTechnique *pTechnique =
              TechniqueManager::GetInstance()->GetTechnique(4);

          RenderLeaf *pRenderLeaf = new RenderLeaf();
          pRenderLeaf->SetDrawable(pLine);
          pRenderLeaf->SetRenderTechnique(pTechnique);

          pTransformNode->AddChild(pRenderLeaf);
        }

        // 方向高亮色带：direction=1 浅蓝 (上山)，direction=2 黄色 (下山)
        if (show_direction_overlay_ && pLaneSegment->GetProperty())
        {
          int dir = pLaneSegment->GetProperty()->direction;
          if (dir == 1 || dir == 2)
          {
            V4f dirCol = (dir == 1) ? V4f(0.40f, 0.70f, 1.00f, 0.60f)
                                    : V4f(1.00f, 0.90f, 0.25f, 0.60f);
            double overlayWidth = 3.0; // 米
            Drawable *pBand = FactoryDrawable::CreateWidthLineDrawable(
                pPolyline, vCnt, overlayWidth, dirCol);
            if (pBand != NULL)
            {
              RenderTechnique *pTechnique =
                  TechniqueManager::GetInstance()->GetTechnique(3);
              RenderLeaf *pRenderLeaf = new RenderLeaf();
              pRenderLeaf->SetDrawable(pBand);
              pRenderLeaf->SetRenderTechnique(pTechnique);
              pTransformNode->AddChild(pRenderLeaf);
            }
          }
        }

        // highlightPoint,
        V3f vclr = pStyle->GetKeyVertexColor();

        Drawable *pNode = FactoryDrawable::CreateNodeDrawable(
            pPolyline, vCnt, point_size_, V4f(vclr[0], vclr[1], vclr[2], 1.0),
            highlightPoint);
        if (pNode != NULL)
        {
          RenderTechnique *pTechnique =
              TechniqueManager::GetInstance()->GetTechnique(1);

          RenderLeaf *pRenderLeaf = new RenderLeaf();
          pRenderLeaf->SetDrawable(pNode);
          pRenderLeaf->SetRenderTechnique(pTechnique);

          pTransformNode->AddChild(pRenderLeaf);
        }

        // if (pPolyline->GetGeometryType() == Geometry::GT_POLYLINE)
        {
          V3f clr = pStyle->GetArrowColor();
          double size = 0.4;
          if ((*it)->GetParallelSegment()->leftSegment > 0 ||
              (*it)->GetParallelSegment()->rightSegment > 0 ||
              (*it)->GetParallelSegment()->leftReverseSegment > 0 ||
              (*it)->GetParallelSegment()->rightReverseSegment > 0 ||
              (*it)->GetSuccessorFeature() || (*it)->GetPredecessorFeature())
            size = 0.8;
          Drawable *pArrow = FactoryDrawable::CreateArrowDrawable(
              pPolyline, vCnt, pStyle->GetLineColor(), size);
          if (pArrow != NULL)
          {
            RenderTechnique *pTechnique =
                TechniqueManager::GetInstance()->GetTechnique(1);

            RenderLeaf *pRenderLeaf = new RenderLeaf();
            pRenderLeaf->SetDrawable(pArrow);
            pRenderLeaf->SetRenderTechnique(pTechnique);

            pTransformNode->AddChild(pRenderLeaf);
          }
        }
      }
    }
  }

  void SegmentLayer::Update(const Matrix4x4f &svMatrix, Camera *pCamera)
  {
    for (std::map<Geometry *, PositionTransformNode *>::iterator it =
             render_leaf_.begin();
         it != render_leaf_.end(); it++)
    {
      PositionTransformNode *pTransformNode = it->second;
      if (pTransformNode != NULL)
      {
        const V3d &position = pTransformNode->GetPosition();

        for (int i = 0; i < pTransformNode->GetNumChildren(); i++)
        {
          RenderLeaf *pRenderLeaf = (RenderLeaf *)pTransformNode->GetChild(i);

          // 生成平移矩阵
          V3d origin = pCamera->GetPostion();
          Viewport viewport = pCamera->GetViewport();

          V3d vecT = position - origin;
          Matrix4x4f mat = Matrix4x4f::MakeTrans(vecT[0], vecT[1], vecT[2]);

          Matrix4x4f mvMatrix = mat * svMatrix;
          Matrix4x4f prjMatrix = pCamera->GetProjectionMatrix();

          // 更新变换矩阵
          pRenderLeaf->SetViewport(viewport);
          pRenderLeaf->SetModelViewMatrix(&mvMatrix);
          pRenderLeaf->SetProjectionMatrix(&prjMatrix);
        }
      }
    }
  }

  void SegmentLayer::Draw(RenderInfo &rendinfo)
  {
    if (!m_bVisible)
      return;
    for (std::map<Geometry *, PositionTransformNode *>::iterator it =
             render_leaf_.begin();
         it != render_leaf_.end(); it++)
    {
      PositionTransformNode *pTransformNode = it->second;
      if (pTransformNode != NULL)
      {
        for (int i = 0; i < pTransformNode->GetNumChildren(); i++)
        {
          RenderLeaf *pRenderLeaf = (RenderLeaf *)pTransformNode->GetChild(i);
          if (pRenderLeaf != NULL)
          {
            pRenderLeaf->Render(rendinfo, NULL);
          }
        }
      }
    }
  }

  struct GEO_HDR
  {
    unsigned int magic;
    unsigned short version;
    unsigned short reserve;
    unsigned int crc32;
  };

  struct GEO_INFO
  {
    int geotype; // 图形类别
    int count;   // 点数量
    double minLat;
    double maxLat;
    double maxLon;
    double minLon;
    float minAlt;
    float maxAlt;
  };

  int SegmentLayer::PackGeometry(Geometry *pPolyline, char *&pMem)
  {
    //---------------------------------------------
    GEO_HDR hdr;

    hdr.magic = 0x00306567;
    hdr.version = 1;
    hdr.reserve = 0;
    hdr.crc32 = 0;

    //------------------------------------

    GEO_INFO info;

    std::vector<Point3d> items;
    Geometry::GeometryType type = pPolyline->GetGeometryType();

    // if (type == Geometry::GT_BEZIER_CURVE) {
    //   GeoBezierCurve3 *pCurve = (GeoBezierCurve3 *)pPolyline;
    //   pCurve->Hermite(items);
    // } else if (type == Geometry::GT_BSPLINE_CURVE) {
    //   GeoBSplineCurve3 *pCurve = (GeoBSplineCurve3 *)pPolyline;
    //   pCurve->Hermite(items);
    // } else if (type == Geometry::GT_CIRCULAR_ARC) {
    //   GeoCircularArc *pCurve = (GeoCircularArc *)pPolyline;
    //   pCurve->Hermite(items);
    // } else if (type == Geometry::GT_HERMITE_CURVE) {
    //   // GeoHermiteCurve3 *pCurve = (GeoHermiteCurve3 *) pPolyline;
    //   // pCurve->Hermite(items);
    // } else if (type == Geometry::GT_POLYLINE || type == Geometry::GT_POLYGON) {
    //   for (int i = 0; i < pPolyline->GetVertexCount(); i++) {
    //     Point3d pt = pPolyline->GetVertex(i);
    //     items.push_back(pt);
    //   }
    // } else if (type == Geometry::GT_RECTANGLE) {
    //   GeoRectangle *pCurve = (GeoRectangle *)pPolyline;
    //   pCurve->Hermite(items);
    // }
    //--------------------------------------------------

    std::vector<GPSPoint> pointlist;
    //-----------------------------------
    int nCount = items.size();
    pointlist.resize(nCount);

    ProjectionUTM projectionUTM;
    for (int i = 0; i < nCount; i++)
    {
      int nzone = ProjectionUTM::zone;
      projectionUTM.CartesianToLatLon(items[i].x, items[i].y, nzone, false,
                                      pointlist[i].latlon);
      pointlist[i].altitude = items[i].z;
    }
    //----------------------------------------------------------

    std::vector<GPSPoint> rawPoints;
    //-----------------------------------------
    int rawCount = pPolyline->GetVertexCount();
    rawPoints.resize(rawCount);

    for (int i = 0; i < rawCount; i++)
    {
      Point3d pt = pPolyline->GetVertex(i);
      int nzone = ProjectionUTM::zone;
      projectionUTM.CartesianToLatLon(pt.x, pt.y, nzone, false,
                                      rawPoints[i].latlon);
      rawPoints[i].altitude = pt.z;
      rawPoints[i].id = pt.GetId();
    }
    // 临时这样写
    hdr.reserve = rawCount;
    //----------------------------------

    info.geotype = type;
    info.count = nCount;
    info.minLat = 0;
    info.maxLat = 0;
    info.maxLon = 0;
    info.minLon = 0;
    info.minAlt = 0;
    info.maxAlt = 0;

    //----------------------------------
    int length = sizeof(GEO_HDR) + sizeof(GEO_INFO) +
                 sizeof(GPSPoint) * (nCount + rawCount);
    char *pMemery = new char[length];
    int nWriteByte = 0;

    memcpy(pMemery + nWriteByte, &hdr, sizeof(GEO_HDR));
    nWriteByte += sizeof(GEO_HDR);

    memcpy(pMemery + nWriteByte, &info, sizeof(GEO_INFO));
    nWriteByte += sizeof(GEO_INFO);

    memcpy(pMemery + nWriteByte, &pointlist[0], sizeof(GPSPoint) * nCount);
    nWriteByte += sizeof(GPSPoint) * nCount;

    memcpy(pMemery + nWriteByte, &rawPoints[0], sizeof(GPSPoint) * rawCount);
    nWriteByte += sizeof(GPSPoint) * rawCount;

    pMem = pMemery;

    return nWriteByte;
  }
} // namespace geditor
