 #include "car.h"

Car::Car(const Map& map_global) :
  map_global_(map_global)
{
}

CarState& Car::GetCarState() {
  return car_state_;
}

const Map& Car::GetGlobalMap() const {
  return map_global_;
}
/*
_____________________________________________________________________________*/
// Performs RK4 integration step and updates state
void Car::UpdateState(const std::vector<float>& control_input_vec,
                      float dt) {
  //const int N_SUBSTEPS = 10 if higher precision needed; run in loop
  float h = dt; // /N_SUBSTEPS
  std::vector<float> k1_vec, k2_vec, k3_vec, k4_vec;
  const std::vector<float>& state_vec = GetCarState().GetStateVector();
  std::vector<float> addition_tmp;
  // k1 =
  EvaluateModel(state_vec, control_input_vec, &k1_vec);
  // k2 =
  AddVectors(1.0, state_vec, 0.5*h, k1_vec, &addition_tmp);
  EvaluateModel(addition_tmp, control_input_vec, &k2_vec);
  // k3 =
  AddVectors(1.0, state_vec, 0.5*h, k2_vec, &addition_tmp);
  EvaluateModel(addition_tmp, control_input_vec, &k3_vec);
  // k4 =
  AddVectors(1.0, state_vec, h, k3_vec, &addition_tmp);
  EvaluateModel(addition_tmp, control_input_vec, &k4_vec);
  // sum up: y_(k+1) = h/6*(k1+2*k2+2*k3+k4)
  AddVectors(h/6., k1_vec, h/3., k2_vec, &addition_tmp);
  AddVectors(1.0, addition_tmp, h/3., k3_vec, &addition_tmp);
  AddVectors(1.0, addition_tmp, h/6., k4_vec, &addition_tmp);
  // add integral to old state vector (x_new = xk_old + dt*xdot)
  AddVectors(1.0, addition_tmp, 1.0, state_vec, &addition_tmp);
  GetCarState().UpdateStateVector(addition_tmp, dt);
  GetCarState().UpdateInputVector(control_input_vec);
}
// Returns xdot = f(x, u); must be overriden by specific car model
// void EvaluateModel(const std::vector<float>& state_vec,
//                    const std::vector<float>& control_input_vec,
//                    std::vector<float>* evaluation_vec) {
// }
// helper routine, to be put somewhere else
/*
_____________________________________________________________________________*/
void Car::AddVectors(float scale1, const std::vector<float>& v1,
                     float scale2, const std::vector<float>& v2,
                std::vector<float>* result) const {
  result->resize(v1.size());
  for (size_t i = 0; i < v1.size(); ++i) {
    (*result)[i] = scale1*v1[i] + scale2*v2[i];
  }
}
/*
______________________________________________________________________________*/
void Car::UpdateLocalMap() {
  std::vector<float> state_vec = GetCarState().GetStateVector();
  map_global_.GetLocalGrid(Pose(state_vec[0], state_vec[1], state_vec[2]),
                           &map_local_);
}

const QImage& Car::GetLocalGrid() const {
//  std::cout << "dim " << map_local_.height() << std::endl;
  return map_local_;
}

/*
 * Set the region in [Meter] the car can look to the side/front
______________________________________________________________________________*/
void Car::SetPovDimension(float width, float height) {
  pov_height_m_ = height;
  pov_width_m_ = width;
  float ppm = map_global_.GetPixelPerMeter();
  QImage tmp_image(pov_width_m_*ppm, pov_height_m_*ppm, QImage::Format_Mono);
  map_local_ = tmp_image;
}
