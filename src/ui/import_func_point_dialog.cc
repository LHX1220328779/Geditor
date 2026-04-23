#include <QFileDialog>
#include <QMessageBox>

#include "algorithm/mc_math.h"
#include "core/geo_rectangle.h"
#include "pcd/db_read_write.h"

#include "import_func_point_dialog.h"
#include "ui_importfuncpointdialog.h"

using namespace geditor;

ImportFuncPointDialog::ImportFuncPointDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ImportFuncPointDialog) {
  ui->setupUi(this);

  QStringList header({"名称,类型,X,Y,Z,方向"});
  model_ = new QStandardItemModel();
  ui->tableView->setModel(model_);

  model_->setColumnCount(6);
  model_->setHeaderData(0, Qt::Horizontal, "名称");
  model_->setHeaderData(1, Qt::Horizontal, "类型");
  model_->setHeaderData(2, Qt::Horizontal, "X");
  model_->setHeaderData(3, Qt::Horizontal, "Y");
  model_->setHeaderData(4, Qt::Horizontal, "Z");
  model_->setHeaderData(5, Qt::Horizontal, "方向");

  ui->tableView->setColumnWidth(0, 70);
  ui->tableView->setColumnWidth(1, 70);
  ui->tableView->setColumnWidth(2, 40);
  ui->tableView->setColumnWidth(3, 40);
  ui->tableView->setColumnWidth(4, 40);
  ui->tableView->setColumnWidth(5, 40);
}

ImportFuncPointDialog::~ImportFuncPointDialog() {
  delete ui;
  delete model_;
}

void ImportFuncPointDialog::on_import_btn_clicked() {
  return QDialog::accept();
}

void ImportFuncPointDialog::on_cancel_btn_clicked() {
  return QDialog::reject();
}

void ImportFuncPointDialog::on_toolButton_clicked() {
  QFileDialog fd(this, "Select file", "", "功能点数据(*.db, *.txt);;");
  if (fd.exec() == QDialog::Accepted) {
    QString sel = fd.selectedFiles()[0];

    //销毁
    for (int i = 0; i < m_jobArray.size(); i++) {
      if (m_jobArray[i]) {
        delete m_jobArray[i];
      }
    }
    m_jobArray.clear();

    //解析
    auto filename = sel.toUtf8().constData();
    ReadDBBoundary(filename, m_boundArray);

    if (ReadDBFunctionPoint(filename, m_jobArray)) {
      ShowData();
      ui->lineEdit->setText(sel);
    } else if (ParseFunctionPoint(filename, m_jobArray)) {
      ShowData();
      ui->lineEdit->setText(sel);
    } else {
      QMessageBox::warning(this, "提示", "功能点解析失败");
    }
  }
}

bool ImportFuncPointDialog::ReadDBBoundary(
    const char *filename, std::vector<geditor::BoundSegment *> &segArray) {
  DBReadWrite db;

  if (!db.OpenDB(filename)) {
    return false;
  }

  std::vector<UniqueType> mArrIdx;
  if (db.QueryAllFunctionPoint(mArrIdx)) {
    //======================================
    struct CurbPair {
      Point3d sPnt;
      Point3d ePnt;
      int type;
    };

    std::map<int, CurbPair> pair_map;

    //=====================================
    for (int i = 0; i < mArrIdx.size(); i++) {
      int type, number;
      double heading;
      V3d pos;

      if (db.ReadFunctionPoint(mArrIdx[i], type, number, pos, heading)) {
        // pos
        UTMPoint utm;
        int curbType;
        bool bStart;

        ProjectionUTM projectUTM;
        int nzone = ProjectionUTM::zone;
        projectUTM.LatLonToCartesian(pos[0], pos[1], utm);

        // 确认贴边线类型
        if (type == 4 || type == 5) {
          // A类
          curbType = 1;
          bStart = (type != 4);
        } else if (type == 6 || type == 7) {
          //  B类
          curbType = 2;
          bStart = (type != 6);
        } else if (type == 8 || type == 9) {
          // C类
          curbType = 3;
          bStart = (type != 8);
        } else if (type == 10 || type == 11) {
          // D类
          curbType = 4;
          bStart = (type != 10);
        } else {
          continue;
        }

        //------------------------------------------
        int unqiue = (curbType << 8) | number;
        std::map<int, CurbPair>::iterator iter_find = pair_map.find(unqiue);
        if (iter_find != pair_map.end()) {
          if (bStart) {
            iter_find->second.sPnt = Point3d(utm.x, utm.y, 0);
          } else {
            iter_find->second.ePnt = Point3d(utm.x, utm.y, 0);
          }
        } else {
          CurbPair curbPair;

          curbPair.type = curbType;
          if (bStart) {
            curbPair.sPnt = Point3d(utm.x, utm.y, 0);
          } else {
            curbPair.ePnt = Point3d(utm.x, utm.y, 0);
          }

          pair_map.insert(std::pair<int, CurbPair>(unqiue, curbPair));
        }
      } else {
        db.CloseDB();
        return false;
      }
    }
    db.CloseDB();

    //============================================

    std::map<int, CurbPair>::iterator iter;
    for (iter = pair_map.begin(); iter != pair_map.end(); iter++) {
      BoundSegment *pLaneSegment = new BoundSegment();

      GeoPolyline *pLine = new GeoPolyline();
      pLine->AppendVertex(iter->second.sPnt);
      pLine->AppendVertex(iter->second.ePnt);

      BoundaryProperty pProperty;
      pProperty.length = 0.0;
      pProperty.boundType = iter->second.type;
      pLaneSegment->SetProperty(&pProperty);

      pLaneSegment->SetGeometry(pLine);
      pLaneSegment->SetUniqueID(iter->first);

      segArray.push_back(pLaneSegment);
    }

    return true;
  }

  return false;
}

bool ImportFuncPointDialog::ReadDBFunctionPoint(
    const char *filename, std::vector<geditor::JobArea *> &jobArray) {
  const char name_map[] = {'A', 'B', 'C', 'D', 'E'};

  DBReadWrite db;
  if (!db.OpenDB(filename)) {
    return false;
  }

  std::vector<UniqueType> mArrIdx;
  if (!db.QueryAllFunctionPoint(mArrIdx)) {
    db.CloseDB();

    return false;
  }

  for (int i = 0; i < mArrIdx.size(); i++) {
    int type, number;
    double heading;
    V3d pos;

    if (db.ReadFunctionPoint(mArrIdx[i], type, number, pos, heading)) {
      JobProperty jobProperty;

      // type
      if (type == 0) {
        // Stop
        jobProperty.areaType = 106;
      } else if (type == 1) {
        // Power
        jobProperty.areaType = 107;
      } else if (type == 2) {
        // Dump
        jobProperty.areaType = 105;
      } else if (type == 3) {
        // Dock
        jobProperty.areaType = 103;
      } else if (type == 4) {
        // water
        jobProperty.areaType = 114;
      } else {
        continue;
      }

      // name
      if (type < 4) {
        sprintf(jobProperty.name, "%c%d", name_map[type], number);
      } else {
        continue;
      }

      // pos
      UTMPoint utm;

      ProjectionUTM projectionUTM;
      projectionUTM.LatLonToCartesian(pos[0], pos[1], utm);
      double thea = Mathd::ToRadians(heading);
      double x = utm.x + Mathd::Cos(thea) * 2.0;
      double y = utm.y + Mathd::Sin(thea) * 2.0;

      GeoRectangle *pGeoRect = new GeoRectangle();
      pGeoRect->AppendVertex(utm.x, utm.y, pos[2]);
      pGeoRect->AppendVertex(x, y, pos[2]);

      JobArea *pJob = new JobArea();
      pJob->SetGeometry(pGeoRect);
      pJob->SetProperty(&jobProperty);

      jobArray.push_back(pJob);
    } else {
      db.CloseDB();
      return false;
    }
  }

  db.CloseDB();

  return true;
}

bool ImportFuncPointDialog::ParseFunctionPoint(
    const char *filename, std::vector<JobArea *> &jobArray) {
  FILE *pfile = fopen(filename, "rb");
  if (!pfile) {
    return false;
  }

  char strLine[1024];
  char delims[] = " ";
  char *record = NULL;

  while (fgets(strLine, 1024, pfile) != NULL) {
    JobProperty jobProperty;

    // type
    record = strtok(strLine, delims);
    if (record != NULL) {
      if (strcmp("Stop", record) == 0) {
        jobProperty.areaType = 106;
      } else if (strcmp("Power", record) == 0) {
        jobProperty.areaType = 107;
      } else if (strcmp("Dump", record) == 0) {
        jobProperty.areaType = 105;
      } else if (strcmp("Dock", record) == 0) {
        jobProperty.areaType = 103;
      } else if (strcmp("Water", record) == 0) {
        jobProperty.areaType = 114;
      } else {
        continue;
      }
    }

    // name
    record = strtok(NULL, delims);
    if (record != NULL) {
      strcpy(jobProperty.name, record);
    } else {
      continue;
    }

    // lat
    double latitude = 0.0;
    record = strtok(NULL, delims);
    if (record != NULL) {
      latitude = atof(record);
    } else {
      continue;
    }

    // lon
    double longitude = 0.0;
    record = strtok(NULL, delims);
    if (record != NULL) {
      longitude = atof(record);
    } else {
      continue;
    }

    // height
    double height = 0.0;
    record = strtok(NULL, delims);
    if (record != NULL) {
      height = atof(record);
    } else {
      continue;
    }

    // angle
    double angle = 0.0;
    record = strtok(NULL, delims);
    if (record != NULL) {
      angle = atof(record);
    } else {
      continue;
    }
    UTMPoint utm;
    ProjectionUTM projectUTM;
    int nzone = ProjectionUTM::zone;
    projectUTM.LatLonToCartesian(latitude, longitude, utm);

    double thea = Mathd::ToRadians(angle);
    double x = utm.x + Mathd::Cos(thea) * 2.0;
    double y = utm.y + Mathd::Sin(thea) * 2.0;

    GeoRectangle *pGeoRect = new GeoRectangle();
    pGeoRect->AppendVertex(utm.x, utm.y, height);
    pGeoRect->AppendVertex(x, y, height);

    JobArea *pJob = new JobArea();
    pJob->SetGeometry(pGeoRect);
    pJob->SetProperty(&jobProperty);

    jobArray.push_back(pJob);
  }

  fclose(pfile);

  return true;
}

void ImportFuncPointDialog::ShowData() {
  model_->removeRows(0, model_->rowCount());
  for (int i = 0; i < m_jobArray.size(); i++) {
    JobArea *pJobArea = m_jobArray[i];

    JobProperty *pProperty = pJobArea->GetProperty();
    Geometry *pGeometry = pJobArea->GetGeometry();

    // 名字
    model_->setItem(i, 0, new QStandardItem(pProperty->name));

    // 类型
    if (pProperty->areaType == 106) {
      model_->setItem(i, 1, new QStandardItem("停车点"));
    } else if (pProperty->areaType == 107) {
      model_->setItem(i, 1, new QStandardItem("维护室"));
    } else if (pProperty->areaType == 105) {
      model_->setItem(i, 1, new QStandardItem("垃圾站"));
    } else if (pProperty->areaType == 103) {
      model_->setItem(i, 1, new QStandardItem("取货点"));
    }

    // 位置
    LatLon latlon;

    ProjectionUTM projectUTM;
    Point3d pnt = pGeometry->GetVertex(0);
    int nzone = ProjectionUTM::zone;
    projectUTM.CartesianToLatLon(pnt.x, pnt.y, nzone, false, latlon);

    model_->setItem(i, 2,
                    new QStandardItem(std::to_string(latlon.lon).c_str()));
    model_->setItem(i, 3,
                    new QStandardItem(std::to_string(latlon.lat).c_str()));
    model_->setItem(i, 4, new QStandardItem(std::to_string(pnt.z).c_str()));

    //方向
    Point3d end1 = pGeometry->GetVertex(1);
    double deltaX = end1.x - pnt.x;
    double deltaY = end1.y - pnt.y;
    double thea = Mathd::Atan2(deltaY, deltaX);

    model_->setItem(
        i, 5,
        new QStandardItem(std::to_string(Mathd::ToDegrees(thea)).c_str()));
  }
}
