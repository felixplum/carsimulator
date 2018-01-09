#include "gui.h"
#include <QApplication>
#include <simulator.h>

int main(int argc, char *argv[])
{
//    CarModelBicycle car_;
//    Map map_;
//    map_.GenerateTestMap();
//    map_.Print(100);
//    Pose pose_(0.7, 0.5, 0.5*3.14);
//    map_.GetCenterPathAtPose(pose_);

  QApplication a(argc, argv);
  GUI w;
  w.setFixedSize(w.size());
  w.show();
  return a.exec();
}
