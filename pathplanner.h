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
  bool GetWaypoints(const QImage& grid_local,
                    std::vector<Point>* waypoint_vec_local_pixel,
                    std::vector<Point>* waypoint_vec_local_meter = NULL,
                    float* pixel_per_m = NULL);
private:
};

#endif // PATHPLANNER_H
