#ifndef CAR_H
#define CAR_H

//#include <cstdio>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <boost/thread/mutex.hpp>
#include "customtypes.h"
#include "QMainWindow" // for QImage
#include "map.h"
//#include <string>

/* CarState encapsulates the state vector and provides methods for access.
   Usage: 1.) car_state["x"] for external processing of state vector (e.g. plotting)
          2.) car_state.GetStateVector() when doing computations
 ____________________________________________________________________________*/
class CarState {
  protected:
  /** Internal states **/
  std::vector<float> state_values_;
  std::vector<float> input_values_;
  std::vector<std::string> state_names_;
  float current_time_;
  bool is_init_ = false;
  boost::mutex rw_mutex_;

  public:
    /** Public interface **/
    void InitState(std::vector<float> state_values_init,
                   std::vector<float> input_values_init,
                   std::vector<std::string> state_names) {
      boost::mutex::scoped_lock(rw_mutex_);
      state_values_ = state_values_init;
      input_values_ = input_values_init;
      state_names_ = state_names;
      current_time_ = 0.;
      is_init_ = true;
      assert(state_values_init.size() == state_names.size());
    }
    void ResetState() {
      boost::mutex::scoped_lock(rw_mutex_);
      if (!is_init_) {
        std::cerr << "Cannot reset car state; state not init." << std::endl;
      }
      state_values_ = std::vector<float>(state_values_.size(), 0.);
      state_values_[0] = SIM::x_init;
      state_values_[1] = SIM::y_init;
      input_values_ = std::vector<float>(input_values_.size(), 0.);
      current_time_ = 0.;
    }
    // returns reference to state
    const std::vector<float>& GetStateVector() const {
      return state_values_;
    }
    float GetTime() const {
      return current_time_;
    }
    const std::vector<std::string>& GetStateNames() const {
      return state_names_;
    }
    const std::vector<float>& GetInputVector() const {
      return input_values_;
    }
    void UpdateStateVector(const std::vector<float>& new_state, float dt) {
      boost::mutex::scoped_lock(rw_mutex_);
      if (is_init_) assert(state_values_.size() == new_state.size());
      state_values_ = new_state;
      current_time_ += dt;
    }
    void SetState(const std::vector<float>& new_state, float time) {
      boost::mutex::scoped_lock(rw_mutex_);
//      if (is_init_) assert(state_values_.size() == new_state.size());
      state_values_ = new_state;
      current_time_ = time;
    }
    void SetCurrentTime(float t) {
      boost::mutex::scoped_lock(rw_mutex_);
      current_time_ = t;
    }
    void UpdateInputVector(const std::vector<float>& new_input) {
      boost::mutex::scoped_lock(rw_mutex_);
      if (is_init_) assert(input_values_.size() == new_input.size());
      input_values_ = new_input;
    }

  /** Overloaded operator **/
  // string access operator
  float operator[](const std::string& state_name) const {
    auto it = std::find(state_names_.begin(),
                        state_names_.end(), state_name);
    if (it != state_names_.end() &&
        state_names_.size() == state_values_.size()) {
      std::ptrdiff_t idx = it - state_names_.begin();
      return state_values_[idx];
    }
    std::cerr << "Requested state does not exist; return 0."
              << std::endl;
    return 0;
  }
  // print operator for debugging
  friend std::ostream& operator<<(std::ostream& os,
                                  const CarState& car_state) {
    boost::mutex::scoped_lock(rw_mutex_);
    for (size_t i = 0; i < car_state.state_values_.size(); ++i) {
      os << car_state.state_names_[i] << ": " << car_state.state_values_[i]
      << std::endl;
    }
    os << "t: " << car_state.current_time_ << std::endl;
    return os;
  }
};

struct CarParams {
public:
  CarParams(){
    description.push_back("Dist. center of mass <-> rear axis [m]");
    value_ptrs.push_back(&lr);
    description.push_back("Dist. center of mass <-> front axis [m]");
    value_ptrs.push_back(&lf);
    description.push_back("Car width [m]");
    value_ptrs.push_back(&width);
    description.push_back("Mass [kg]");
    value_ptrs.push_back(&mass);
    description.push_back("Max. steering angle [deg]");
    value_ptrs.push_back(&steering_max_deg);
    description.push_back("Max. velocity [m/s]");
    value_ptrs.push_back(&v_max);
  }
  std::vector<std::string> description;
  std::vector<float*> value_ptrs;
  float lr = 0.5*CONSTANTS::CAR_LENGTH_METER;
  float lf = 0.5*CONSTANTS::CAR_LENGTH_METER;
  float width = CONSTANTS::CAR_WIDTH_METER;
  float mass = CONSTANTS::CAR_MASS_KG;
  float steering_max_deg = 45.;
  float v_max = 0.7;
};

// add names for car models here:
enum CAR_TYPE {CT_BICYCLE};

/*** Main class declaration ****
 ____________________________________________________________________________*/

class Car {
  public:
    Car(const Map& map_global);
    CarParams params_;
    CarState& GetCarState();
    void UpdateLocalMap();
    // Performs RK4 integration step and updates state
    void UpdateState(const std::vector<float>& control_input_vec,
                     float dt);
    void SetPovDimension(float width, float height);
    const QImage& GetLocalGrid() const;
    virtual void GetControl(std::vector<float>* u_out) = 0;
  protected:
    // Returns xdot = f(x, u); must be overriden by specific car model
    virtual void EvaluateModel(const std::vector<float>& state_vec,
                               const std::vector<float>& control_input_vec,
                               std::vector<float>* evaluation_vec) const = 0;

    const Map& GetGlobalMap() const;
  private:
    void AddVectors(float scale1, const std::vector<float>& v1,
                    float scale2, const std::vector<float>& v2,
                    std::vector<float>* result) const;
    CarState car_state_;
    const Map& map_global_;
    QImage map_local_;
    float pov_width_m_;
    float pov_height_m_;
};

typedef std::shared_ptr<Car> CarPtr;

#endif // CAR_H
