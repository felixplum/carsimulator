#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "customtypes.h"
#include "map.h"
#include "QMainWindow"
#include "car.h"


class PathPlanner
{
public:
  PathPlanner();
  bool GetWaypoints(QImage* grid_local,
                    float pixel_per_m,
                    std::vector<Point>* waypoint_vec_local);
private:
};

#endif // PATHPLANNER_H
