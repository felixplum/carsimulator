#include "simulator.h"

Simulator::Simulator(float dt) :
  SAMPLE_TIME_(dt),
  simulation_run_state_(RS_STOPPED) {
}

/* main thread loop
___________________________________________________________________________*/
void Simulator::Run() {
  while (simulation_run_state_ == RS_RUNNING) {
    boost::this_thread::sleep_for(
          boost::chrono::milliseconds(static_cast<int>(SAMPLE_TIME_*1e3)));
    UpdateCars();
    //std::cout << "This thread is running lol" << std::endl;
  }
}

/*
_____________________________________________________________________________*/
void Simulator::AddNewCar(CAR_TYPE car_model_type) {
  switch(car_model_type) {
    case CT_BICYCLE: {
      // explicit ctor for unique_ptr
      CarPtr car(new CarModelBicycle);
      // only one owner of unique_ptr, use move semantics
      simulated_cars_.push_back(car);
      // todo: create state_history for each car
      std::cout << "Added new car of type CT_BICYCLE" << std::endl;
      break;
    }
    default:
      std::cerr << "Couldn't add car; invalid type" << std::endl;
      break;
  }
}

/* Controls must be supplied externally,
   either by offline stored vals. or controls
____________________________________________________________________________*/

void Simulator::UpdateCars() {
  for (auto& car_it : simulated_cars_) {
    // either call ApplyControlStep() to let car decide control input
    // or call UpdateStep() for applying external input (e.g. stored in file)
      std::vector<float> u = {1., 0.1}; //  v=1, steering = 0.
      car_it->UpdateState(u, SAMPLE_TIME_);
      std::cout << car_it->GetCarState() << std::endl;
  }
}

/* this fct. spawn new run thread, resets car states if necessary
_____________________________________________________________________________*/
void Simulator::ChangeRunStatus(RunState new_state) {
  //boost::mutex::scoped_lock(simulation_state_mutex_);
  switch(new_state) {
    case RS_RUNNING: {
      simulation_run_state_ = RS_RUNNING;
      boost::thread t(&Simulator::Run, this);
      break;
    }
    case RS_PAUSED:
    case RS_STOPPED:
    case RS_READY:
    default: std::cerr << "ChangeRunStatus: Default case.." << std::endl;
  }
}

// attachInputTimeSeriesToCar
