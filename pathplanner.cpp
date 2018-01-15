#include "pathplanner.h"

PathPlanner::PathPlanner() {
}

/*
 * This method sample local grid and computes center line stored as waypoints
__________________________________________________________________________*/
bool PathPlanner::GetWaypoints(const QImage& grid_local,
                               std::vector<Point>* waypoint_vec_local_pixel,
                               std::vector<Point>* waypoint_vec_local_meter,
                               float* pixel_per_m) {
  size_t n_cols = grid_local.width();
  size_t n_rows = grid_local.height();
  int n_waypoints = 10;
  int row_increment = n_rows/n_waypoints;
  waypoint_vec_local_pixel->clear();
  // start at top of the grid and iterate rows downwards:
  for (size_t row_idx = 0; row_idx < n_rows; row_idx += row_increment) {
    bool found_left = false;
    bool found_right = false;
    Point left_border_pnt;
    Point right_border_pnt;

    size_t col_idx_start = 0.5*n_cols;
    // init. search at previous center
    if (!waypoint_vec_local_pixel->empty()) {
      col_idx_start = waypoint_vec_local_pixel->back().y;
    }
    // scan columns from center to left:
    for (int col_idx = col_idx_start; col_idx >= 0; --col_idx) {
      QColor c = QColor::fromRgb(grid_local.pixel(col_idx, row_idx));
      if (c == Qt::black || col_idx == 0) {
        // break if init. point is black:
        if (col_idx == col_idx_start) {break;};
        // else point is valid border point
        left_border_pnt = Point(row_idx, col_idx);
        found_left = true;
        break;
      }
    }
    // scan columns from center to right:
    for (size_t col_idx = col_idx_start; col_idx < n_cols; ++col_idx) {
      QColor c = QColor::fromRgb(grid_local.pixel(col_idx, row_idx));
      if (c == Qt::black || col_idx == n_cols-1) {
        if (col_idx == col_idx_start) {break;};
        right_border_pnt = Point(row_idx, col_idx);
        found_right = true;
        break;
      }
    }
    // if left and right border point found, take mean
    // and add to waypoint vector:
    Point center_pnt;
    if (found_left && found_right) {
      center_pnt.x = 0.5*(left_border_pnt.x + right_border_pnt.x);
      center_pnt.y = 0.5*(left_border_pnt.y + right_border_pnt.y);
      waypoint_vec_local_pixel->push_back(center_pnt);
//      printf("added wp %f %f", center_pnt.x, center_pnt.y);
//      std::cout<<std::endl;
    }
  }
  // check if conversion to car frame required:
  if (waypoint_vec_local_meter && pixel_per_m) {
    waypoint_vec_local_meter->clear();
    for (size_t i = 0; i < waypoint_vec_local_pixel->size(); ++i) {
        // transform from local pixel coord. to metric values in local car frame:
        Point center_pnt = (*waypoint_vec_local_pixel)[i];
        center_pnt.x = (n_rows-center_pnt.x)/(*pixel_per_m);
        center_pnt.y = (0.5*n_cols-center_pnt.y)/(*pixel_per_m);
        waypoint_vec_local_meter->push_back(center_pnt);
    }
  }

  return !waypoint_vec_local_pixel->empty();

}

