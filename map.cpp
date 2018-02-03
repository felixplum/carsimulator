#include "map.h"

const double PI  = 3.141592653589793238463;

Map::Map(const QPixmap& pixmap)
{
  // convert to QImage for efficient access
  grid_ = pixmap.toImage();
  // 1 bit image, occupied = black = 0
  grid_.convertToFormat(QImage::Format_Mono);
  n_rows_ = grid_.height();
  n_cols_ = grid_.width();
}

/*
_____________________________________________________________________________*/
//void Map::GenerateTestMap() {
//  n_rows_ = 50;
//  n_cols_ = 100;
//  meter_per_index_ = 3./100.; // 1.5mx3m map
//  occupancy_grid_.reserve(n_rows_*n_cols_);
//  for (int irow = 0; irow < n_rows_; ++irow) {
//    for (int icol = 0; icol < n_cols_; ++icol) {
//      int val = (irow > 0.4*n_rows_ && irow < 0.6*n_rows_)?0:1;
//      occupancy_grid_[irow + icol*n_rows_] = val;
//    }
//  }
//}
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
/* swap x and y!
_____________________________________________________________________________*/
//int Map::GetGridValueAtMetricPoint(const Point& target) const {
//  int col_idx = target.x / (CONSTANTS::X_MAX_METER) * n_cols_;
//  int row_idx = (1-target.y / (CONSTANTS::Y_MAX_METER)) * n_rows_;
////  std::cout << "access at point " << target.x << " " << target.y << std::endl;
////  std::cout << "access at row/col " << row_idx << " " << col_idx << std::endl;
//  if ((row_idx < 0) || (row_idx > n_rows_-1) ||
//      (col_idx < 0) || (col_idx > n_cols_-1)) {
//    std::cerr << "GetGridValueAtMetricPoint tried to access invalid index"
//              << std::endl;
//    return 1;
//  }
//  QColor c = QColor::fromRgb(grid_.pixel(row_idx, col_idx));
//  return c==Qt::black?1:0;
////  return grid_.pixel(row_idx, col_idx);
//}

/* This method returns a grid (image) from the point of view
of the car, which looks upwards from the bottom center, e.g.
the car's x-axis pointing upwards */
void Map::GetLocalGrid(const Pose& pose_curr, QImage* local_grid) const {
//  static boost::mutex access_mutex;

//  boost::mutex::scoped_lock lock(access_mutex);

  float sin_car = sin(-pose_curr.phi);
  float cos_car = cos(-pose_curr.phi);
  // car's origin in global pixel coord.
  int col_idx0 = pose_curr.x / (CONSTANTS::X_MAX_METER) * n_cols_;
  int row_idx0 = (1-pose_curr.y / (CONSTANTS::Y_MAX_METER)) * n_rows_;

  int n_cols_local = local_grid->width();
  int n_rows_local = local_grid->height();
  // fill to remove artifacts
//  local_grid->fill(1);
  // iterate over pixels in local grid
  for (int rowidx = 0; rowidx < n_rows_local; rowidx++) {
    for (int colidx = 0; colidx < n_cols_local; colidx++) {
      // Transform row/col of local grid into pixel coord. in car frame:
      float x_car = n_rows_local - rowidx;
      float y_car = 0.5*n_cols_local - colidx;
      // Transform into global frame (rotate + shift):
      float col_g = x_car*cos_car - y_car*sin_car + col_idx0;
      float row_g = x_car*sin_car + y_car*cos_car + row_idx0;
      // Check boundary
      if ((row_g < 0) || (row_g > n_rows_-1) ||
          (col_g < 0) || (col_g > n_cols_-1)) {
        // mark out-of-bounds-pixel as 0/obstacle/black
        local_grid->setPixel(n_cols_local-colidx-1, rowidx, 0);
        continue;
      }
      // lookup pixel value in global map and fill into local one
      QColor c = QColor::fromRgb(grid_.pixel(col_g, row_g));
      local_grid->setPixel(n_cols_local-colidx-1, rowidx, c==Qt::white?1:0);
    }
  }
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

void Map::SetMap(const QPixmap& pixmap) {
   grid_ = pixmap.toImage();
   n_rows_ = grid_.height();
   n_cols_ = grid_.width();
}
  // convert to QImage for efficient access


/* todo: iterate over multiple points, go in both directions, do basic testing */
//std::vector<Point> Map::GetCenterPathAtPose(const Pose& pose_current) {
//  std::vector<Point> result_path;
//  bool border1_found = false;
//  float normal_slope = tan(pose_current.phi+0.5*PI);
//  // step of dx leads to dy = dx*normal_slope
//  // => dx = 1/sqrt(1+slope^2) lead to 1 unit distance
//  // this step then leads to an increase of one index value:
//  float dx_index_step = meter_per_index_/sqrt(1+pow(normal_slope, 2));
//  // go into pependicular direction on grid, find border
//  // go one index in x-direction, and calculate the y_index
//  float x_border = pose_current.x;
//  float y_border = pose_current.y;
//  int i = 1;
//  Point border_point(pose_current.x, pose_current.y);
//  while (!border1_found && i < 100) {
//    float dx = i*dx_index_step;
//    border_point.x += dx;
//    border_point.y = pose_current.y + dx*normal_slope;
//    border1_found = GetGridValueAtMetricPoint(border_point);
//    i++;
//  }
//  if (border1_found) {
//    printf("Found border point at (x, y) = (%f, %f)\n", border_point.x, border_point.y);
//    Pose local_pose(0,0,0);
//    Pose border_pose(border_point.x, border_point.y, -1);
//    TransformPoseintoSystem(border_pose, pose_current, &local_pose);
//    printf("Point at (x, y) in local coord. = (%f, %f)\n", local_pose.x, local_pose.y);
//  } else {
//    std::cout << "Failed to find border point :(" << std::endl;
//    std::cout << "normal_slope is " << normal_slope << std::endl;
//    std::cout << "dx_index_step is " << dx_index_step << std::endl;
//  }
//  return result_path;
//}

// void Map::UnitTest() {

// }

/**/

float Map::GetPixelPerMeter() const {
  return grid_.width()/CONSTANTS::X_MAX_METER;
}
