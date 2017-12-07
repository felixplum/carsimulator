#include "gui.h"
#include <QApplication>
#include <car_model_bicycle.h>
#include <simulator.h>
#include <map.h>

int main(int argc, char *argv[])
{
//    CarModelBicycle car_;
//    Map map_;
//    map_.GenerateTestMap();
//    map_.Print(100);
//    Pose pose_(0.7, 0.5, 0.5*3.14);
//    map_.GetCenterPathAtPose(pose_);
    Simulator sim_(0.1);
    sim_.AddNewCar();
    sim_.ChangeRunStatus(RS_RUNNING);
    QApplication a(argc, argv);
    GUI w;
    w.show();
    return a.exec();
}
