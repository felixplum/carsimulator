#include "car_model_bicycle.h"

CarModelBicycle::CarModelBicycle(const Map& map_global) : Car(map_global) {
  std::vector<float> init_state = {1.05,0,0};
  std::vector<float> init_input = {0,0};
  std::vector<std::string> state_names = {"x","y","phi"};  
  GetCarState().InitState(init_state, init_input, state_names);
}
/*
_____________________________________________________________________________*/
/* Model according to
  http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf */
void CarModelBicycle::EvaluateModel(const std::vector<float>& state_vec,
                                    const std::vector<float>& control_input_vec,
                                    std::vector<float>* evaluation_vec) const {
  static float lr = 0.05;  // distance from center of mass to front axis
  static float lf = 0.05;  // .. to back axis
  // global pose:
  float phi = state_vec[2];
  float v_input = control_input_vec[0];
  float steering_input = control_input_vec[1];
  evaluation_vec->resize(3);
  // angle of velocity of center of mass rel. to longitudinal axis 
  float beta = atan(lr/(lr+lf)*tan(steering_input));
  (*evaluation_vec)[0] = v_input*cos(phi + beta);
  (*evaluation_vec)[1] = v_input*sin(phi + beta);
  (*evaluation_vec)[2] = v_input/lr*sin(beta);
}

void CarModelBicycle::GetControl(std::vector<float>* u_out) {
  UpdateWaypoints();
}

// Waypoints should be plotted later on, therefore use member variable
/*
____________________________________________________________________________*/
bool CarModelBicycle::UpdateWaypoints() {
  const QImage& grid_local = GetLocalGrid();
  size_t n_cols = grid_local.width();
  size_t n_rows = grid_local.height();
  int n_waypoints = 10;
  int row_increment = n_rows/n_waypoints;
  waypoint_vec_local_pixel_.clear();
  // start at top of the grid and iterate rows downwards:
  for (size_t row_idx = 0; row_idx < n_rows; row_idx += row_increment) {
    bool found_left = false;
    bool found_right = false;
    Point left_border_pnt;
    Point right_border_pnt;

    size_t col_idx_start = 0.5*n_cols;
    // init. search at previous center
    if (!waypoint_vec_local_pixel_.empty()) {
      col_idx_start = waypoint_vec_local_pixel_.back().y;
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
      waypoint_vec_local_pixel_.push_back(center_pnt);
//      printf("added wp %f %f", center_pnt.x, center_pnt.y);
//      std::cout<<std::endl;
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
void CarModelBicycle::GetWaypointsPixel(std::vector<Point>* wp_out) const {
  *wp_out = waypoint_vec_local_pixel_;
}
