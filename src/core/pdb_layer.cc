#include "core/pdb_layer.h"
#include "core/line_drawable.h"
#include "core/render_leaf.h"

#include "map/pdb_manage.h"
#include "map/projection_utm.h"
#include "map/tile_tool.h"

#include "utils/color_interpolation.h"

#include "renderGL/mc_render_pass.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_technique_manager.h"

namespace geditor {

PDBLayer::PDBLayer()
    : Layer(LT_PDB),
      m_database(nullptr),
      m_minValue(0),
      m_maxValue(255),
      m_minHigh(-50),
      m_maxHigh(50),
      m_loader(nullptr) {}

void PDBLayer::SetDataSource(const char *szName) {
  /// 加载线程
  if (m_loader != NULL) {
    m_loader->cancel();
  } else {
    Callback *callback = new Callback(&m_RenderLeaf);
    m_loader = new Loader();
    m_loader->setCallback(callback);
  }
  m_loader->SetColorType(color_type_, color_r_);

  //------------------------------
  // 数据源处理
  if (m_database != NULL) {
    m_database->Close();
  } else {
    m_database = new PDBManage();
  }
  //-----------------------------

  // 启动加载线程
  if (m_database->Open(szName)) {
    m_loader->SetDataFile(m_database);
    m_loader->start();
  }
}

bool PDBLayer::GetBoundary(V3d &minVec, V3d &maxVec) {
  if (!m_database) {
    return false;
  }

  ProjectionUTM projectionUTM;

  GPSPoint minGPSPt, maxGPSPt;

  if (m_database->QueryBound(minGPSPt, maxGPSPt)) {
    UTMPoint point;
    projectionUTM.LatLonToCartesian(minGPSPt.latlon.lat, minGPSPt.latlon.lon,
                                    point);
    minVec = V3d(point.x, point.y, 0.0);

    projectionUTM.LatLonToCartesian(maxGPSPt.latlon.lat, maxGPSPt.latlon.lon,
                                    point);
    maxVec = V3d(point.x, point.y, 0.0);

    return true;
  }

  return false;
}

PDBLayer::~PDBLayer() {
  if (m_loader) {
    m_loader->cancel();
    delete m_loader;
    m_loader = NULL;
  }

  if (m_database) {
    m_database->Close();

    delete m_database;
    m_database = NULL;
  }

  for (std::map<TileGrid, PositionTransformNode *>::iterator it =
           m_RenderLeaf.begin();
       it != m_RenderLeaf.end(); it++) {
    PositionTransformNode *pTransformNode = it->second;
    if (pTransformNode != NULL) {
      delete pTransformNode;
    }
  }
  m_RenderLeaf.clear();
}

void PDBLayer::CloseDataSource() {
  if (m_loader) {
    m_loader->cancel();
    delete m_loader;
    m_loader = NULL;
  }

  if (m_database) {
    m_database->Close();
    delete m_database;
    m_database = NULL;
  }

  m_RenderTile.clear();

  for (std::map<TileGrid, PositionTransformNode *>::iterator it =
           m_RenderLeaf.begin();
       it != m_RenderLeaf.end(); it++) {
    PositionTransformNode *pTransformNode = it->second;
    if (pTransformNode != NULL) {
      delete pTransformNode;
    }
  }
  m_RenderLeaf.clear();
}

void PDBLayer::SetPointCloudFilter(float minVal, float maxVal) {
  m_minValue = minVal;
  m_maxValue = maxVal;
}

void PDBLayer::GetPointCloudFilter(float &minVal, float &maxVal) {
  minVal = m_minValue;
  maxVal = m_maxValue;
}

void PDBLayer::SetPointHighFilter(float minVal, float maxVal) {
  m_minHigh = minVal;
  m_maxHigh = maxVal;
}

void PDBLayer::GetPointHighFilter(float &minVal, float &maxVal) {
  minVal = m_minHigh;
  maxVal = m_maxHigh;
}

void PDBLayer::SetColorType(int type, int r) {
  color_type_ = type;
  color_r_ = r;
  if (m_loader) m_loader->SetColorType(color_type_, color_r_);
}
void PDBLayer::GetColorType(int &type, int &r) {
  type = color_type_;
  r = color_r_;
}

void PDBLayer::Cull(double minX, double minY, double maxX, double maxY,
                    float scale) {
  ProjectionUTM projectionUTM;

  LatLon maxLaton;
  int nzone = ProjectionUTM::zone;
  projectionUTM.CartesianToLatLon(maxX, maxY, nzone, false, maxLaton);
  TileGrid maxGrid;
  TileTools::LatLon2Grid(maxLaton.lat, maxLaton.lon, maxGrid);

  LatLon minLaton;
  projectionUTM.CartesianToLatLon(minX, minY, nzone, false, minLaton);
  TileGrid minGrid;
  TileTools::LatLon2Grid(minLaton.lat, minLaton.lon, minGrid);

  //---------------------------------------------------------
  // 根据需要显示效果，调整下面分层参数，会影响渲染效果
  //     0.125       0.0625       0.03125    0.015625
  // 0层 ------- 1层 ------- 2层 ------- 3层 ------- 4层
  float map_scale[] = {0.125, 0.0625, 0.03125,
                       0.015625};  //���ƹ�5�㣬��4���ű����߷ֿ�

  int zoom = 0;

  int iSize = sizeof(map_scale) / sizeof(float);
  for (int i = iSize - 1; i >= 0; i--) {
    if (map_scale[i] > scale) {
      zoom = i + 1;
      break;
    }
  }

  //----------------------------------------

  //�ü�����ǰ��������Ԫ��
  std::vector<TileGrid> tileArray;
  for (int y = minGrid.y; y <= maxGrid.y; y++) {
    for (int x = minGrid.x; x <= maxGrid.x; x++) {
      for (int z = 4; z >= zoom; z--) {
        tileArray.push_back(TileGrid(x, y, z));
      }
    }
  }

  if (m_loader) {
    m_RenderTile.clear();
    for (int i = 0; i < tileArray.size(); i++) {
      std::map<TileGrid, PositionTransformNode *>::iterator iter_find =
          m_RenderLeaf.find(tileArray[i]);
      if (iter_find != m_RenderLeaf.end()) {
        if (iter_find->second) {
          m_RenderTile.push_back(iter_find->second);
        }
      } else {
        m_loader->Add(tileArray[i]);
      }
    }
  }
}

void PDBLayer::Update(const Matrix4x4f &svMatrix, Camera *pCamera) {
  for (std::vector<PositionTransformNode *>::iterator it = m_RenderTile.begin();
       it != m_RenderTile.end(); it++) {
    PositionTransformNode *pTransformNode = *it;
    if (pTransformNode != NULL) {
      const V3d &position = pTransformNode->GetPosition();
      for (int i = 0; i < pTransformNode->GetNumChildren(); i++) {
        RenderLeaf *pRenderLeaf = (RenderLeaf *)pTransformNode->GetChild(i);
        if (pRenderLeaf) {
          //����ƽ�ƾ���
          V3d origin = pCamera->GetPostion();

          V3d vecT = position - origin;
          Matrix4x4f mat = Matrix4x4f::MakeTrans(vecT[0], vecT[1], vecT[2]);

          Matrix4x4f mvMatrix = mat * svMatrix;
          Matrix4x4f prjMatrix = pCamera->GetProjectionMatrix();

          //���±任����
          RenderTechnique *pTechnique =
              TechniqueManager::GetInstance()->GetTechnique(0);
          RenderPass *pRenderPass = pTechnique->GetPass(0);
          Program *pProgram = pRenderPass->GetShaderProgram();

          UniformParam *param = pProgram->GetUniformParam();

          param->SetParameter(2, V3f(m_minHigh, m_maxHigh, 0.0f));
          param->SetParameter(3, V3f(m_minValue, m_maxValue, 0.0f));

          pRenderLeaf->SetRenderTechnique(pTechnique);
          pRenderLeaf->SetModelViewMatrix(&mvMatrix);
          pRenderLeaf->SetProjectionMatrix(&prjMatrix);
        }
      }
    }
  }
}

void PDBLayer::Draw(RenderInfo &rendinfo) {
  if (!m_bVisible) return;
  for (std::vector<PositionTransformNode *>::iterator it = m_RenderTile.begin();
       it != m_RenderTile.end(); it++) {
    PositionTransformNode *pTransformNode = *it;
    if (pTransformNode) {
      for (int i = 0; i < pTransformNode->GetNumChildren(); i++) {
        RenderLeaf *pRenderLeaf = (RenderLeaf *)pTransformNode->GetChild(i);
        if (pRenderLeaf != NULL) {
          pRenderLeaf->GetDrawable()->SetColorType(color_type_, color_r_);
          pRenderLeaf->Render(rendinfo, NULL);
        }
      }
    }
  }
}

}  // namespace geditor
