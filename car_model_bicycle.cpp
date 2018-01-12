#include "car_model_bicycle.h"

CarModelBicycle::CarModelBicycle() {
  std::vector<float> init_state = {0,0,0};
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

void CarModelBicycle::GetControl(const std::vector<Point>& waypoints,
                                 std::vector<float>* u_out) const {

}
