#include <QApplication>

#include "core/log.h"
#include "geditor_mainwindow.h"

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 0;
  QApplication a(argc, argv);
  GeditorMainWindow w;
  w.show();

  return a.exec();
}
