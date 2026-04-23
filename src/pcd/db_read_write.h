
#pragma once

#include <map>
#include <string>
#include <vector>

#include "algorithm/common.h"

typedef int UniqueType;

namespace geditor {
typedef struct Pose {
  V3d pos;
  Quatd quat;
} DBPose;

class DBReadWrite {
 public:
  bool OpenDB(const char *szfileName);

  bool ReOpen();

  void CloseDB();

  bool IsOpen() const;

  //--------------------------------------------------------

  bool WriteMoveOriginPoint(double x, double y, double z, int zone,
                            bool bEast = false);

  bool ReadMoveOriginPoint(V3d &orgin, int &zone, bool &bEast);

  // -------------------------------------------------------

  bool WriteHistogramBlock(int x, int y, const void *pData, int nSize);

  bool ReadHistogramBlock(int x, int y, char *&pData, int &length);

  // -------------------------------------------------------

  bool WriteScanContextBlock(int x, int y, const void *pData, int nSize);

  bool ReadScanContextBlock(int x, int y, char *&pData, int &length);

  //---------------------------------------------------------

  bool WriteRawLaserData(UniqueType uniqueId, const DBPose &pose,
                         const void *pData, int length, int version);

  bool ReadRawLaserData(UniqueType uniqueId, DBPose &pose, void *&pData,
                        int &length, int &version);

  //---------------------------------------------------------

  bool WriteOptimizePoseData(UniqueType uniqueId, const DBPose &pose);

  bool ReadOptimizePoseData(UniqueType uniqueId, DBPose &pose);

  //----------------------------------------------------------

  bool WriteFunctionPoint(UniqueType uniqueId, int type, int number,
                          const V3d &pos, double heading);

  bool ReadFunctionPoint(UniqueType uniqueId, int &type, int &number, V3d &pos,
                         double &heading);

  //--------------------------

  bool ReadTrackLine(UniqueType uniqueId, char *&pData, int &length);

  //--------------------------

  bool ReadTrackImage(UniqueType uniqueId, char *&pData, int &length);

  //--------------------------

  bool QueryScanContextUnique(std::vector<std::pair<int, int>> &arrI);

  bool QueryHistogramUnique(std::vector<std::pair<int, int>> &arrI);

  bool QueryAllRawLaserUnique(std::vector<UniqueType> &arrIdxPose);

  bool QueryAllOptimizePose(std::map<int, DBPose> &mMapIdxPose);

  bool QueryAllTrackLine(std::vector<UniqueType> &mTrackIdx);

  bool QueryAllFunctionPoint(std::vector<UniqueType> &mIdxArray);

  //--------------------------

  bool QueryBoundary(V3d &vMin, V3d &vMax);

  bool QueryRawLaserData(const V3d &position, double radius,
                         std::vector<UniqueType> &arrayId);

 private:
  bool Unpack_PointCloud(char *szData, int length, int &version,
                         void *&pPackBuf, int &nPackLenth);

  bool Pack_PointCloud(const void *szData, int length, int version,
                       char *&pPackBuf, int &nPackLenth);

 private:
  bool ReadPoseData();

 private:
  std::map<int, DBPose> map_idx_pose_;
  std::string file_name_;
};

}  // namespace geditor
