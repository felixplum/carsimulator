#include "simulator.h"

Simulator::Simulator(float dt_sample,
                     QPixmap& global_grid) :
  dt_sample_(dt_sample),
  simulation_run_state_(RS_STOPPED),
  map_(global_grid)
{
}

/* main thread loop
___________________________________________________________________________*/
void Simulator::Run() {
  while (simulation_run_state_ == RS_RUNNING) {
    boost::this_thread::sleep_for(
          boost::chrono::milliseconds(static_cast<int>(dt_sample_*1e3)));
    UpdateCars();
    //std::cout << "This thread is running lol" << std::endl;
  }
}

/*
_____________________________________________________________________________*/
CarPtr Simulator::AddNewCar(CAR_TYPE car_model_type) {
  switch(car_model_type) {
    case CT_BICYCLE: {
      // explicit ctor for unique_ptr
      CarPtr car(new CarModelBicycle(map_));
      car->SetPovDimension(1., 1.);
      // only one owner of unique_ptr, use move semantics
      simulated_cars_.push_back(car);
      // todo: create state_history for each car
      std::cout << "Added new car of type CT_BICYCLE" << std::endl;
      return car;
      break;
    }
    default:
      std::cerr << "Couldn't add car; invalid type" << std::endl;
      return nullptr;
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
//      std::vector<float> u = {0.6, 0.1}; //  v=1, steering = 0.
      std::vector<float> u = {0.5, 0};
      car_it->GetControl(&u);
      car_it->UpdateState(u, dt_sample_);
      car_it->UpdateLocalMap();
//      std::cout << "u: " << u[1] << std::endl;
//      std::cout << car_it->GetCarState() << std::endl;
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
      simulation_run_state_ = new_state;
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


/* Returns ref. to local grid for first car (use for display!)
_____________________________________________________________________________ */
const QImage& Simulator::GetLocalGrid() const {
  if(simulated_cars_.empty()) {
    std::cerr << "Cannot return local grid; no cars avaiblable" << std::endl;
  } else {
    return (simulated_cars_[0]->GetLocalGrid());
  }
// return map_local_;
}

// For display purposes:
void Simulator::GetWaypointsPixel(std::vector<Point>* wp_out) const {
  if(simulated_cars_.empty()) {
    std::cerr << "Cannot return local waypoints; no cars available" << std::endl;
  }
  CarModelBicycle* bm = dynamic_cast<CarModelBicycle*>(simulated_cars_[0].get());
  bm->GetWaypointsPixel(wp_out);

}


// SetSampleTime()

// attachInputTimeSeriesToCar
