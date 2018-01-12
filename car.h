#ifndef CAR_H
#define CAR_H

//#include <cstdio>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <boost/thread/mutex.hpp>
#include "customtypes.h"
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
    const std::vector<float>& GetInputVector() const {
      return input_values_;
    }
    void UpdateStateVector(const std::vector<float>& new_state, float dt) {
      boost::mutex::scoped_lock(rw_mutex_);
      state_values_ = new_state;
      current_time_ += dt;
    }
    void UpdateInputVector(const std::vector<float>& new_input) {
      boost::mutex::scoped_lock(rw_mutex_);
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

enum CAR_TYPE {CT_BICYCLE};

/*** Main class declaration ****
 ____________________________________________________________________________*/

class Car {
  public:
    Car();
    CarState& GetCarState();
    // Performs RK4 integration step and updates state
    void UpdateState(const std::vector<float>& control_input_vec,
                     float dt);
  protected:
    /* Methods that need to be overwritten by specific car
       implementation: */
    // Returns xdot = f(x, u); must be overriden by specific car model
    virtual void EvaluateModel(const std::vector<float>& state_vec,
                               const std::vector<float>& control_input_vec,
                               std::vector<float>* evaluation_vec) const = 0;
    virtual void GetControl(const std::vector<Point>& waypoints,
                       std::vector<float>* u_out) const = 0;
  private:
    void AddVectors(float scale1, const std::vector<float>& v1,
                    float scale2, const std::vector<float>& v2,
                    std::vector<float>* result) const;
    CarState car_state_;
};

#endif // CAR_H
