#include "map.h"

const double PI  =3.141592653589793238463;

Map::Map() {
}
/*
_____________________________________________________________________________*/
void Map::GenerateTestMap() {
  n_rows_ = 50;
  n_cols_ = 100;
  meter_per_index_ = 3./100.; // 1.5mx3m map
  occupancy_grid_.reserve(n_rows_*n_cols_);
  for (int irow = 0; irow < n_rows_; ++irow) {
    for (int icol = 0; icol < n_cols_; ++icol) {
      int val = (irow > 0.4*n_rows_ && irow < 0.6*n_rows_)?0:1;
      occupancy_grid_[irow + icol*n_rows_] = val;
    }
  }
}
/*
_____________________________________________________________________________*/
void Map::Print(int max_cols) {
  size_t max_c = std::min(n_cols_, max_cols);
  for (int irow = 0; irow < n_rows_; ++irow) {
    for (size_t icol = 0; icol < max_c; ++icol) {
      std::cout << occupancy_grid_[irow + icol*n_rows_];
    }
    std::cout << std::endl;
  }
}
/*
_____________________________________________________________________________*/
int Map::GetGridValueAtMetricPoint(const Point& target) const {
  int row_idx = target.x / meter_per_index_;
  int col_idx = target.y / meter_per_index_;  
  if ((row_idx < 0) || (row_idx > n_rows_-1) ||
      (col_idx < 0) || (col_idx > n_cols_-1)) {
    std::cerr << "GetGridValueAtMetricPoint tried to access invalid index"
              << std::endl;
    return -1e9;
  }
  return occupancy_grid_[row_idx + col_idx*n_rows_];
}
/*
_____________________________________________________________________________*/
// given a input pose and a new coord. frame (both in global coord.), compute input pose in 
// given system's coordinates
void Map::TransformPoseintoSystem(const Pose& pose_input, const Pose& frame_new,
                                 Pose* pose_in_new_frame) {
  float dx_global = frame_new.x - pose_input.x;
  float dy_global = frame_new.y - pose_input.y;
  float dphi = frame_new.phi - pose_input.phi;
  pose_in_new_frame->x = dx_global * cos(dphi) - dy_global * sin(dphi);
  pose_in_new_frame->y = dx_global * sin(dphi) + dy_global * cos(dphi);
  pose_in_new_frame->phi = dphi;
}

/* todo: iterate over multiple points, go in both directions, do basic testing */
std::vector<Point> Map::GetCenterPathAtPose(const Pose& pose_current) {
  std::vector<Point> result_path;
  bool border1_found = false;
  float normal_slope = tan(pose_current.phi+0.5*PI);
  // step of dx leads to dy = dx*normal_slope
  // => dx = 1/sqrt(1+slope^2) lead to 1 unit distance
  // this step then leads to an increase of one index value:
  float dx_index_step = meter_per_index_/sqrt(1+pow(normal_slope, 2));
  // go into pependicular direction on grid, find border
  // go one index in x-direction, and calculate the y_index
  float x_border = pose_current.x;
  float y_border = pose_current.y;
  int i = 1;
  Point border_point(pose_current.x, pose_current.y);
  while (!border1_found && i < 100) {
    float dx = i*dx_index_step;
    border_point.x += dx;
    border_point.y = pose_current.y + dx*normal_slope;
    border1_found = GetGridValueAtMetricPoint(border_point);
    i++;
  }
  if (border1_found) {
    printf("Found border point at (x, y) = (%f, %f)\n", border_point.x, border_point.y);
    Pose local_pose(0,0,0);
    Pose border_pose(border_point.x, border_point.y, -1);
    TransformPoseintoSystem(border_pose, pose_current, &local_pose);
    printf("Point at (x, y) in local coord. = (%f, %f)\n", local_pose.x, local_pose.y);
  } else {
    std::cout << "Failed to find border point :(" << std::endl;
    std::cout << "normal_slope is " << normal_slope << std::endl;    
    std::cout << "dx_index_step is " << dx_index_step << std::endl;
  }
  return result_path;
}

// void Map::UnitTest() {

// }

/**/
