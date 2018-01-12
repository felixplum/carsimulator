#include "pathplanner.h"

PathPlanner::PathPlanner() {
}

/*
 * This method sample local grid and computes center line stored as waypoints
__________________________________________________________________________*/
bool PathPlanner::GetWaypoints(QImage* grid_local,
                               float pixel_per_m,
                               std::vector<Point>* waypoint_vec_local) {
  int n_cols = grid_local->width();
  int n_rows = grid_local->height();
  int n_waypoints = 5;
  int row_increment = n_rows/n_waypoints;
  waypoint_vec_local->clear();
  // start at top of the grid and iterate rows downwards:
  for (int row_idx = 0; row_idx < n_rows; row_idx += row_increment) {
    bool found_left = false;
    bool found_right = false;
    Point left_border_pnt;
    Point right_border_pnt;
    // from col. center, iterate columns to left and right simultaneously:
    // scan left:
    for (int i = 0; (i < 0.5*n_cols) && !(found_left&&found_right); ++i) {
      if(!found_left) {
        int col_idx = 0.5*n_cols-i-1;
        QColor c = QColor::fromRgb(grid_local->pixel(col_idx, row_idx));
        if (c == Qt::black || col_idx == 0) {
          left_border_pnt = Point(row_idx, col_idx);
          found_left = true;
        }
      }
      // scan right:
      if(!found_right) {
        int col_idx = 0.5*n_cols+i;
        QColor c = QColor::fromRgb(grid_local->pixel(col_idx, row_idx));
        if (c == Qt::black || col_idx == n_cols-1) {
          right_border_pnt = Point(row_idx, col_idx);
          found_right = true;
        }
      }
    }
    // if left and right border point found, take mean
    // and add to waypoint vector:
    Point center_pnt;
    if (found_left && found_right) {
      center_pnt.x = 0.5*(left_border_pnt.x + right_border_pnt.x);
      center_pnt.y = 0.5*(left_border_pnt.y + right_border_pnt.y);
      waypoint_vec_local->push_back(center_pnt);
    }
    // transform from local pixel coord. to metric values in local car frame:
//    center_pnt.x = (n_rows-center_pnt.x)/pixel_per_m;
//    center_pnt.y = (0.5*n_cols-center_pnt.y)/pixel_per_m;
//      printf("added wp %f %f", center_pnt.x, center_pnt.y);
//      std::cout<<std::endl;
  }
}

