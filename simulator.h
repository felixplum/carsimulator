#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "car.h"
#include "car_model_bicycle.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <memory> // for smart ptr
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>

/*This class handles all simulation objects, peforms state updates,
  provides interface to car objects*/

enum RunState {RS_READY, RS_RUNNING, RS_PAUSED, RS_STOPPED};

typedef std::shared_ptr<Car> CarPtr;

class Simulator {
 public:
   Simulator(float dt_sample);
   // to be called from GUI
   CarPtr AddNewCar(CAR_TYPE car_model_type = CT_BICYCLE);
   void UpdateCars();
   // called when pressing Start/Resume
   void ChangeRunStatus(RunState new_state);
   void ResetState();
 private:
   // all cars to be simulated stored here
   void Run();
   std::vector<CarPtr> simulated_cars_;
   std::vector<std::vector<CarState>> state_histories_; // make own class
   float dt_sample_;
   RunState simulation_run_state_;
   boost::mutex simulation_state_mutex_;
};
#endif // SIMULATOR_H
