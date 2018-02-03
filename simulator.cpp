#include "simulator.h"

Simulator::Simulator(float dt_sample) :
  dt_sample_(dt_sample),
  simulation_run_state_(RS_STOPPED)
{}


/* main thread loop
___________________________________________________________________________*/
void Simulator::Run() {
  while (simulation_run_state_ == RS_RUNNING) {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(dt_sample_*1e3)));
    UpdateCars();
//    std::cout << "This thread is running lol" << std::endl;
  }
}

/*
_____________________________________________________________________________*/
//CarPtr Simulator::AddNewCar(CAR_TYPE car_model_type) {
//  switch(car_model_type) {
//    case CT_BICYCLE: {
//      // explicit ctor for unique_ptr
//      CarPtr car(new CarModelBicycle(map_));
//      car->SetPovDimension(1., 1.);
//      // only one owner of unique_ptr, use move semantics
//      simulated_cars_.push_back(car);
//      // todo: create state_history for each car
//      std::cout << "Added new car of type CT_BICYCLE" << std::endl;
//      return car;
//      break;
//    }
//    default:
//      std::cerr << "Couldn't add car; invalid type" << std::endl;
//      return nullptr;
//      break;
//  }
//}
void Simulator::AddCarPtr(CarPtr car_ptr) {
  simulated_cars_.push_back(car_ptr);
}

bool Simulator::RemoveCarPtr(CarPtr car_ptr) {
  for (size_t i = 0; i < simulated_cars_.size(); ++i) {
    if (car_ptr == simulated_cars_[i]) {
      simulated_cars_.erase(simulated_cars_.begin()+i);
      std::cout << "Removed car from simulation" << std::endl;
      return true;
    }
  }
  std::cout << "Could not removed car from simulation - not found" << std::endl;
  return false;
}

/* Controls must be supplied externally,
   either by offline stored vals. or controls
____________________________________________________________________________*/

void Simulator::UpdateCars() {
//  RunState state = simulation_run_state_;
//  if (state == RS_RUNNING)
//    ChangeRunStatus(RS_STOPPED);
  for (auto& car_it : simulated_cars_) {
      if (!car_it) continue;
      std::vector<float> u;
      car_it->GetControl(&u);
      car_it->UpdateState(u, dt_sample_);
      car_it->UpdateLocalMap();
  }
}

/* this fct. spawn new run thread, resets car states if necessary
_____________________________________________________________________________*/
void Simulator::ChangeRunStatus(RunState new_state) {
  //boost::mutex::scoped_lock(simulation_state_mutex_);
  switch(new_state) {
    case RS_RUNNING: {
      simulation_run_state_ = RS_RUNNING;
      if (!thread_) thread_.reset(new std::thread(&Simulator::Run, this));
      break;
    }
    case RS_PAUSED:
    case RS_STOPPED:
    case RS_READY:
      simulation_run_state_ = new_state;
      if (thread_) {thread_->join(); thread_.reset();}
      break;
    default: std::cerr << "ChangeRunStatus: Default case.." << std::endl;
  }
}
/* Reset
_____________________________________________________________________________*/
void Simulator::ResetState() {
  ChangeRunStatus(RS_STOPPED);
  for (auto& car_it : simulated_cars_) {
      car_it->GetCarState().ResetState();
  }
}

/*
______________________________________________________________________________
______________________________________________________________________________
_________________ Methods for displaying; todo: give car index _______________*/



// SetSampleTime()

// attachInputTimeSeriesToCar
