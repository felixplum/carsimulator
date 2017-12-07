#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cmath>

/* -----------> y (cols)
   |
   |
   |
   |
   x (rows) */

struct Pose {
  public:
    Pose(float x_in, float y_in, float phi_in) : x(x_in), y(y_in), phi(phi_in) {};
    float x;
    float y;
    float phi;
};

struct Point {
  public:
    Point(float x_in, float y_in) : x(x_in), y(y_in) {};
    float x;
    float y;
};

class Map {
  public:
    Map();
    void LoadFromFile(std::string file_name,
                      float x_max_in_m, float y_max_in_m);
    void GenerateTestMap();
    void Print(int max_cols);
    std::vector<Point> GetCenterPathAtPose(const Pose& pose_current);
    int GetGridValueAtMetricPoint(const Point& target) const;
    void TransformPoseintoSystem(const Pose& pose_input, const Pose& frame_new,
                                 Pose* pose_in_new_frame);
    void UnitTest();
  private:
    std::vector<int> occupancy_grid_; // in row major
    int n_rows_;
    int n_cols_;
    float meter_per_index_;
    
    
};

#endif // MAP_H
