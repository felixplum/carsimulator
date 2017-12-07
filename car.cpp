 #include "car.h"

Car::Car() {}

CarState& Car::GetCarState() {
    return car_state_;
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
  GetCarState().UpdateInputVector(addition_tmp);
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
