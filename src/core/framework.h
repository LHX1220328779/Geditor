
#pragma once

#include <stack>
#include <string>
#include <vector>

#include "core/arc_ball.h"
#include "core/camera.h"
#include "core/geometry.h"
#include "core/layer.h"
#include "core/map_feature.h"
#include "core/object.h"
#include "core/render_info.h"

// #include "core/bound_segment.h"
// #include "core/job_layer.h"

#include "algorithm/matrix44.h"
#include "algorithm/viewport.h"
#include "map/feature_type.h"

namespace geditor {

class PointLayer;
class PDBLayer;
class SegmentLayer;
class BoundaryLayer;
class AreaLayer;
class JobLayer;
class TopologyLayer;
class TrajectoryLayer;
class VDBManage;
class IMapCommand;
class BoundSegment;
class CurbsTrack;
class JobArea;
class LaneSegment;
struct Lane;

class Framework : public Object {
 public:
  Framework();

  ~Framework();

 public:
  void Init();

  //缩放接口
  void Zoom(float fzoom);

  void Set3DView();

  void Set2DView();

  void setMapCenter(double lat, double lon);

  void setMapCenterUTM(double x, double y, double z = 0.0);

  //平滑移动接口
  void ViewPanMap();

  void Pan(double dx, double dy);

  void LButton(float x, float y);

  void MouseMove(float x, float y);

  void MouseRotate(float x, float y);

  void KeyMove(float x, float y);

  void GeneratePathPoint();

  //画线接口
  bool StartNewGeoemtry(DrawType type, double fx, double fy, bool llink);

  void StartNewPoint(DrawType type, double fx, double fy);

  bool StartNewLine(DrawType type, double fx, double fy, bool llink);

  bool StartNewBoundary(DrawType type, double fx, double fy, bool llink);

  void StartNewArea(DrawType type, double fx, double fy, bool llink);

  bool StartJobArea(DrawType type, double fx, double fy, bool llink);

  void StartNewElement(DrawType type, double fx, double fy);

  void DrawPointToGeoemtry(double fx, double fy);

  void EndDrawGeoemtry();

  bool IsEndGeoemtry();

  bool DeleteDrawGeomtry();

  void SetPointCloudFilter(float minVal, float maxVal);

  bool GetPointCloudFilter(float &minVal, float &maxVal);

  void SetPointHighFilter(float minVal, float maxVal);

  bool GetPointHighFilter(float &minVal, float &maxVal);

  void SetColorType(int type, int r);
  bool GetColorType(int &type, int &r);

  void SetGridSize(double x, double y);

  double GetGridSize();

  void SetLineWidth(double x);

  double GetLineWidth();

  double GetPointSize();

  void SetPointSize(double x);

  bool GetCarBodyCheck();

  void SetCarBodyCheck(bool bCheck);

  // road_right 可视化：road_right=1 的中心线两侧填充显示为浅绿色
  void SetVizRoadRight(bool on);

  bool GetVizRoadRight();

  // 方向高亮 overlay：选中"设定行驶方向"时开启
  void SetShowDirectionOverlay(bool on);

  bool GetShowDirectionOverlay();

  //道路元素编辑
  void ReverseSegment();

  bool OptimizeCurve3(double x, double y, int &counter);

  //高亮对象
  void HighlightSegment(double x, double y, int type);
  int HighlightPoint(double &x, double &y);

  //选择对象
  void SelectSegment(bool mulselect, bool bLink, double x, double y);

  bool SelectTrajectoryImage(double x, double y, MapFeature *&pFeature,
                             int &keyPoint);

  //插入点
  void InsertPoint(double x, double y);

  //边界填充点
  void FillPointsOnSelectedBoundary(double interval);

  //打断
  void BreakSegment(double x, double y);

  //测距
  void BeginMeasureDistance(double x, double y);

  void MoveMeasureDistance(double x, double y);

  void EndMeasureDistance();

  double GetMeasureDistance() const;

  //整体移动地图要素
  void MoveMapObject(double x, double y);

  void EndMoveObject();

  //移动控制点
  void MoveSegment(double x, double y);

  void EndMovePoint();

  void AddTrajectory(std::vector<BoundSegment *> segArray);

  void AddTrajectory(std::vector<Lane *> segArray);

  void AddBoundary(std::vector<Lane *> segArray);

  void AddBoundary(std::vector<BoundSegment *> segArray);

  void AddSegment(std::vector<Lane *> segArray);

  void AddJobArea(std::vector<JobArea *> segArray);

  void BeginCliperPolygon(double x, double y);

  void MoveCliperPolygon(double x, double y);

  void EndCliperPolygon();

  bool GenerateSweepArea();

  void MergeObject();

  void ConvertToBoundary(int type);

  int SetLaneBoundary();

  void SetPathPointEdge(bool bDelete);

  bool GenerateSweepPath(int type, int mouseX, int mouseY);

  void DeleteMapFeature();

  int SetRegionCurb();

  int SetParallelSegment();
  int SetReverseSegment();

  int SetSegmentRelation();

  int SetTrafficStopline();

  void GenerateTopology();
  int ReverseObj();

  void ActiveLayer(LayerType type);

  LayerType GetActiveLayer() const;

  bool ShowHideTrackLayer(bool show);
  bool ShowHideAreaLayer(bool show);
  bool ShowHideBoundaryLayer(bool show);
  bool ShowHideJobLayer(bool show);
  bool ShowHideSegmentlayerLayer(bool show);
  bool ShowHidePDBLayer(bool show);
  bool ShowHideSignLayer(bool show);

  int ReadCrubImage(char *&blockData, int &length);

  int ReadPreCrubImage(MapFeature *pFeature, int keyPoint, char *&blockData,
                       int &length);

  int ReadNextCrubImage(MapFeature *pFeature, int keyPoint, char *&blockData,
                        int &length);

  bool ReadCrubImage(int keyPoint, char *&blockData, int &length);

  bool ReadCrubsTrack(std::vector<CurbsTrack *> &segmentArray);

  bool LoadDBMap(const char *fileName);

  void CloseDBMap();

  MapFeature *GetMapFeature();

  MapFeature *GetTrajecMapFeature();

  void ClearTrajecMapFeature();

  void SetTrajecKeyPoint(int nKeyPoint);

  LaneSegment *GetLaneSegment();

  void UndoOperate();

  void RedoOperate();

  //保存
  bool IsOpen();

  bool Create(const char *filename);

  bool Read(const char *filename);

  void Close();

  void Save();

  void SetDataSource(const char *szFileName);

  void CloseDataSource();

  void Resize(int x, int y, int width, int height);

  void Draw();

  void Destroy();

  //坐标拾取接口
  void MousePointToCart(int x, int y, double &dx, double &dy);

  int CreateLaneByBoundary();
  LaneSegment *CreateLaneByBoundary(BoundSegment *b0, BoundSegment *b1);
  int CreateReverseLaneGroup();
  int CreateReverseLaneGroup2();
  int CreateBoundaryByLane();
  std::vector<MapFeature *> GetSelected() { return m_selectedSet; }

  int CheckRelation();

  void SetLaneConf(float wl, float wm) {
    w_lane_ = wl;
    w_middle_ = wm;
  }
  void GetLaneConf(float &wl, float &wm) {
    wl = w_lane_;
    wm = w_middle_;
  }

  void ChangeLineType(int type);

  // 自动将所有中心线绑定到重合度最高的轨迹。
  // distTol: 走廊宽度(米)，点落入该距离内视为覆盖。
  // coverageThreshold: 最低覆盖率(0~1)，低于则不绑定。
  // dryRun: true 只计算不写入属性。
  // 返回成功绑定(或可绑定)的车道数量。
  // 诊断信息通过输出参数 diag 回填 (可为空)。
  struct AutoBindDiagnostic {
    int totalLanes = 0;             // 中心线总数
    int unboundLanes = 0;           // 未绑定的中心线数
    int refTotal = 0;               // 总参考轨迹数
    int refFromTrajectoryLayer = 0; // 来自 TrajectoryLayer
    int refFromBoundaryLayer = 0;   // 来自 BoundaryLayer
    int refFromSegmentLayer = 0;    // 来自 SegmentLayer
    int bound = 0;                  // 本次绑定成功数
    std::string detail;             // 每条车道的匹配详情(多行文本)
  };

  int AutoBindTrajectory(double distTol, double coverageThreshold,
                         bool dryRun, AutoBindDiagnostic *diag);

  // ========== 快捷功能 ==========

  // 拾取最近的轨迹折线节点：返回命中的 BoundSegment* 及其顶点索引；
  // 未命中返回 false。tolerance 为屏幕像素换算后的世界坐标距离(米)。
  bool PickTrajectoryAnchor(double x, double y, double tolerance,
                            BoundSegment *&segOut, int &vertexIdxOut);

  // 根据同一条轨迹上的起点/终点锚(顶点索引)，截取轨迹节点生成一条新的中心线，
  // 方向由 idxStart -> idxEnd 决定。中心线会继承该轨迹的 mineSegmentIndex/Code。
  // 返回新 LaneSegment*，失败返回 nullptr。
  LaneSegment *GenerateCenterlineFromAnchors(BoundSegment *traj, int idxStart,
                                             int idxEnd);

  // 设定行驶方向链式生成：
  // direction=1 上山 / 2 下山。
  // 以 (segA,idxA) 为起点、(segB,idxB) 为终点，在轨迹图(按端点几何相邻)中找到
  // 一条轨迹段序列 s0,s1,...,sk（s0=segA, sk=segB，相邻段端点几何距离 < eps）；
  // 每段独立调用内部生成逻辑产出一条新的 LaneSegment（保留段级 1:1 拓扑），
  // 并串联设置 predecessor/successor；所有生成的 lane 会被赋 direction。
  // 返回生成的 lane 数量；失败(无路径/相同轨迹-相同索引) 返回 0。
  struct DirectionChainDiagnostic {
    int chainSegments = 0;       // 找到的轨迹段数量
    int generatedLanes = 0;      // 实际生成的中心线数量
    std::string detail;
  };
  int GenerateCenterlineChainByDirection(BoundSegment *segA, int idxA,
                                         BoundSegment *segB, int idxB,
                                         int direction,
                                         DirectionChainDiagnostic *diag);

  // 根据阈值 A 自动判定所有中心线的 road_right：
  // 整条中心线上每个点到左右边界线的最小距离都 > threshold -> road_right=1；
  // 否则 road_right=0。返回被更新的中心线数量。
  struct RoadRightDiagnostic {
    int totalLanes = 0;
    int compliant = 0;      // road_right=1
    int nonCompliant = 0;   // road_right=0
    int missingBoundary = 0;// 缺少左/右边界
    std::string detail;
  };
  int AutoCheckRoadRight(double threshold, RoadRightDiagnostic *diag);

  // 校验中心线连通性：起始端只有后继；终止端只有前继；中间两者都有。
  // 汇总诊断文本返回给UI。
  struct ConnectivityDiagnostic {
    int totalLanes = 0;
    int starts = 0;   // 起始端(只有后继)
    int ends = 0;     // 终止端(只有前继)
    int middles = 0;  // 中间(都有)
    int isolated = 0; // 孤立(都没有)
    std::vector<int> isolatedIds;
    std::string detail;
  };
  int CheckLaneConnectivity(ConnectivityDiagnostic *diag);

  // 导入/导出 road_right.txt (每行: mineSegmentCode road_right)。
  // Export: 遍历所有中心线，按 mineSegmentCode 去重取第一个。
  // Import: 按 mineSegmentCode 查找中心线并回填 road_right，返回匹配/未匹配数。
  struct RoadRightIOStat {
    int written = 0;    // export
    int matched = 0;    // import
    int unmatched = 0;  // import
    std::string detail;
  };
  bool ExportRoadRightTxt(const std::string &path, RoadRightIOStat *stat);
  bool ImportRoadRightTxt(const std::string &path, RoadRightIOStat *stat);

  // 自适应边界生成参数（R3 + R4）
  struct AdaptiveBoundaryParams {
    // 所有 "车道宽" = 中心线到边界的距离（即车道半宽）。
    // 双开均匀碾压·最低车道宽：d/2 > min_lane_width_dual 时启用双开
    double min_lane_width_dual = 5.8;
    // 双开均匀碾压·最高车道宽：halfW 封顶（顶到时放宽内-内 gap 约束）
    double max_lane_width_dual = 7.0;
    // 单开均匀碾压·最高车道宽：下山优先填到该值；上山取剩余 d-该值-gap
    double max_lane_width_single = 5.8;
    // 左右边界与理论宽度的收缩间距（内-内默认距离）
    double boundary_gap = 0.2;
    // 段间平滑区间（米）：两端各该值内把宽度平滑过渡到接缝最小宽
    double taper_len = 10.0;
  };
  struct AdaptiveBoundaryDiagnostic {
    int totalLanes = 0;
    int processed = 0;
    int dualWide = 0;    // 双开均匀碾压段
    int singleWide = 0;  // 单开均匀碾压段
    int skipped = 0;     // 无对向/无法求 d 的段
    std::string detail;
  };
  // 对所有中心线按参数重新生成左右边界；旧边界会被替换。
  // 相邻段通过前驱/后继关系在接缝处做 taper 平滑。
  int GenerateAdaptiveBoundaries(const AdaptiveBoundaryParams &params,
                                 AdaptiveBoundaryDiagnostic *diag);
  // 局部模式：仅对 targetLanes 列表中的中心线重建边界；
  // 其他中心线保持不变；相邻段接缝仍尽量贴合已有邻居。
  int GenerateAdaptiveBoundariesForSelected(
      const AdaptiveBoundaryParams &params,
      const std::vector<LaneSegment *> &targetLanes,
      AdaptiveBoundaryDiagnostic *diag);

 private:
  V3f m_StartMouse;

  TrackBall m_trackball;

  StereoCamera m_StereoCamera;
  PlanCamera m_PlanCamera;
  Camera *m_pCamera;

  // Matrix4x4f         m_scaleMatrix;

  PointLayer *m_pointLayer;
  AreaLayer *m_areaLayer;
  PDBLayer *m_pdbLayer;
  SegmentLayer *m_Segmentlayer;
  BoundaryLayer *m_boundlayer;
  JobLayer *m_joblayer;
  TopologyLayer *m_topologylayer;
  TrajectoryLayer *m_trajectorylayer;

  std::vector<Layer *> layers_;

  //撤销，重做
  std::stack<IMapCommand *> m_commands;
  std::stack<IMapCommand *> m_histroys;

  //当前回执的对象
  MapFeature *m_pDrawFeature;

  //高亮对象
  MapFeature *m_pHighFeature;

  //选择集
  std::vector<MapFeature *> m_selectedSet;

  //选择的关键点
  MapFeature *m_selectFeature;
  int m_keyPoint;

  //关联对象

  //选择的关键点---轨迹线
  MapFeature *m_selectTrajecFeature = NULL;
  int m_keyTrajecPoint;

  Layer *m_layer;
  LayerType m_layertype;

  // float              m_fScale;
  float m_fMapScale;

  RenderInfo m_rendinfo;
  Viewport m_viewport;

  VDBManage *m_vdbmgr;

  float w_middle_ = 0.5, w_lane_ = 7.5;

  Geometry::GeometryType geoline_type_;
};

}  // namespace geditor
