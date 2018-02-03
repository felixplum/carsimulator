#include "car_model_bicycle_dynamic.h"


CarModelBicycleDynamic::CarModelBicycleDynamic(const Map& map_global) : Car(map_global) {
  // state: x, y, phi, xdot, ydot
  std::vector<float> init_state = {SIM::x_init, SIM::y_init, 0, 0};
  std::vector<float> init_input = {0,0};
  last_steering_ = 0;
  std::vector<std::string> state_names = {"x","y","phi", "v"};
  GetCarState().InitState(init_state, init_input, state_names);
}
/*
_____________________________________________________________________________*/
/* Model according to
  http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf */
void CarModelBicycleDynamic::EvaluateModel(const std::vector<float>& state_vec,
                                    const std::vector<float>& control_input_vec,
                                    std::vector<float>* evaluation_vec) const {
  // global pose:
  float phi = state_vec[2];
  float v = state_vec[3];
  float force_input = control_input_vec[0];
  float steering_input = control_input_vec[1];
  evaluation_vec->resize(4);
  // angle of velocity of center of mass rel. to longitudinal axis
  float beta = atan(params_.lr/(params_.lr+params_.lf)*tan(steering_input));
  (*evaluation_vec)[0] = v*cos(phi + beta);
  (*evaluation_vec)[1] = v*sin(phi + beta);
  (*evaluation_vec)[2] = v/params_.lr*sin(beta);
  (*evaluation_vec)[3] = force_input / params_.mass - 2*v*v; // + drag force
}

void CarModelBicycleDynamic::GetControl(std::vector<float>* u_out) {
  u_out->resize(2);
  UpdateWaypoints();
  size_t wp_idx = 1;//0.5*waypoint_vec_local_meter_.size();
  if (waypoint_vec_local_meter_.size()<=wp_idx) {
      (*u_out)[1] = last_steering_;
      return;
  }
  Point& wp = waypoint_vec_local_meter_[wp_idx];
  float u_tmp = std::atan2(wp.y, wp.x);
  float steering_rad_max = params_.steering_max_deg/(CONSTANTS::RAD2DEG);
  // set steering angle:
  (*u_out)[1] = std::max(1.*std::min(1.*u_tmp, 1.* steering_rad_max),
                        -1.* steering_rad_max);
  last_steering_ = (*u_out)[1];
  // set force:
  // too large angle to last waypoint -> deccelarate
//  float dx = waypoint_vec_local_meter_.back().x - waypoint_vec_local_meter_.front().x;
//  float dy = waypoint_vec_local_meter_.back().y - waypoint_vec_local_meter_.front().y;
//  float angle_ratio = fabs(atan2(dy, dx))/steering_rad_max;
  (*u_out)[0] = 0.03; // [N];
//  (*u_out)[0] += -0.01*angle_ratio;
}

// Waypoints should be plotted later on, therefore use member variable
/*
____________________________________________________________________________*/
bool CarModelBicycleDynamic::UpdateWaypoints() {
  const QImage& grid_local = GetLocalGrid();
  size_t n_cols = grid_local.width();
  size_t n_rows = grid_local.height();
  int n_waypoints = 10;
  int row_increment = n_rows/n_waypoints;
  waypoint_vec_local_pixel_.clear();
  // start at top of the grid and iterate rows downwards:
  for (size_t row_idx_tmp = 0; row_idx_tmp < n_rows; row_idx_tmp += row_increment) {
    size_t row_idx = n_rows - row_idx_tmp - 1;
    bool found_left = false;
    bool found_right = false;
    Point left_border_pnt;
    Point right_border_pnt;

    size_t col_idx_start = 0.5*n_cols;
    // init. search at previous center
    int n_size = waypoint_vec_local_pixel_.size();
    if (n_size == 1) {
       // take prev. column to start
      col_idx_start = waypoint_vec_local_pixel_.back().y;
    } else if (n_size > 1) {
      // linearly extrapolate
      float dx = waypoint_vec_local_pixel_[n_size-1].x -
                 waypoint_vec_local_pixel_[n_size-2].x;
      float dy = waypoint_vec_local_pixel_[n_size-1].y -
                 waypoint_vec_local_pixel_[n_size-2].y;
      col_idx_start = waypoint_vec_local_pixel_[n_size-1].y +
          dy/dx * row_increment;
    }
    // scan columns from center to left:
    for (int col_idx = col_idx_start; col_idx >= 0; --col_idx) {
      QColor c(QColor::fromRgb(grid_local.pixel(col_idx, row_idx)));
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
      QColor c(QColor::fromRgb(grid_local.pixel(col_idx, row_idx)));
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
      waypoint_vec_local_pixel_.push_back(center_pnt);
    }
  }
  waypoint_vec_local_meter_.clear();
  float pixel_per_m = GetGlobalMap().GetPixelPerMeter();
  for (size_t i = 0; i < waypoint_vec_local_pixel_.size(); ++i) {
      // transform from local pixel coord. to metric values in local car frame:
      Point center_pnt = waypoint_vec_local_pixel_[i];
      center_pnt.x = (n_rows-center_pnt.x)/pixel_per_m;
      center_pnt.y = (0.5*n_cols-center_pnt.y)/pixel_per_m;
      waypoint_vec_local_meter_.push_back(center_pnt);
  }

  return !waypoint_vec_local_pixel_.empty();
}

// For display purposes:
void CarModelBicycleDynamic::GetWaypointsPixel(std::vector<Point>* wp_out) const {
  *wp_out = waypoint_vec_local_pixel_;
}
