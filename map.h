#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cmath>
#include "customtypes.h"
#include <QMainWindow>
#include <boost/thread/mutex.hpp>

// storage in grid:
/* -----------> (cols)
   |
   |
   |
   |
   (rows) */


// world coordinates:

/* | y
   |
   |
   | ---------> (x)
 */

// mapping from world coord. to indices required!


class Map {
  public:
    Map(const QPixmap& pixmap);
    void GenerateTestMap();
    void Print(int max_cols);
    std::vector<Point> GetCenterPathAtPose(const Pose& pose_current);
    int GetGridValueAtMetricPoint(const Point& target) const;
    void TransformPoseintoSystem(const Pose& pose_input, const Pose& frame_new,
                                 Pose* pose_in_new_frame);
    void GetLocalGrid(const Pose& pose_curr, QImage* local_grid) const;
    float GetPixelPerMeter() const;
    void UnitTest();
    void SetMap(const QPixmap& pixmap);
  private:
    QImage grid_;
    std::vector<int> occupancy_grid_; // in row major
    int n_rows_;
    int n_cols_;
    float pixel_per_meter_;
    
    
};

#endif // MAP_H
