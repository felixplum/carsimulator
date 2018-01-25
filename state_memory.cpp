#include "state_memory.h"

StateMemory::StateMemory() {
  is_recording_ = false;
}

//void StateMemory::SetCallback(std::function<void()> callback) {

//}
/*
_____________________________________________________________________________*/
void StateMemory::RunRecording() {
  while (is_recording_) {
    // stop simulation, synch? callback/signal
    boost::this_thread::sleep_for(
          boost::chrono::milliseconds(static_cast<int>(dt_sample_*1e3)));
    for (size_t i = 0; i < car_vec_.size(); ++i) {
      if (records_vec_[i]->record_type_ == Record::RecordType::RT_READ_FROM) {
          continue;
      }
      records_vec_[i]->AddState(car_vec_[i]->GetCarState().GetStateVector(),
                                car_vec_[i]->GetCarState().GetTime());
    }
  }
  std::cout << "Recording stopped" << std::endl;
}
/* Adds a record which is preloaded from file
_____________________________________________________________________________*/
//void StateMemory::AddReadRecord(CarPtr car, const std::string& file_name) {
//  std::cerr << "READFROM FILE method still missing!!" << std::endl;
//  /* to: read tab delimited file [x y phi] [u_1 u_2]*/
//  RecordPtr new_record(new Record(car->GetCarState().GetStateVector().size(),
//                       Record::RecordType::RT_READ_FROM));
//  records_vec_.push_back(std::move(new_record));
//  car_vec_.push_back(car);
//}
/*
_____________________________________________________________________________*/
void StateMemory::AddWriteRecord(CarPtr car) {
    RecordPtr new_record(new Record(car->GetCarState().GetStateVector().size(),
                         Record::RecordType::RT_WRITE_TO));
  records_vec_.push_back(std::move(new_record));
  car_vec_.push_back(car);
}

bool StateMemory::RemoveRecordPtr(CarPtr car_ptr) {
  for (size_t i = 0; i < car_vec_.size(); ++i) {
    if (car_ptr == car_vec_[i]) {
      car_vec_.erase(car_vec_.begin()+i);
      records_vec_.erase(records_vec_.begin() + i);
      std::cout << "Removed car + record from state memory" << std::endl;
      return true;
    }
  }
  std::cout << "Could not removed car + record from state memory - not found" << std::endl;
  return false;
}
/*
_____________________________________________________________________________*/
void StateMemory::ToggleRecording(bool activate_recording, float dt_sample) {
  dt_sample_ = dt_sample;
  if (!is_recording_) {
    is_recording_ = true;
    boost::thread t(&StateMemory::RunRecording, this);
  } else if (!activate_recording) {
    is_recording_ = false;
  }
}

/*
_____________________________________________________________________________*/
std::vector<RecordPtr> StateMemory::GetRecordPtrVec() const {
  return records_vec_;
}

/* Delete records, but keep number of records const.
_____________________________________________________________________________*/
void StateMemory::ResetState() {
  ToggleRecording(false);
  for (size_t i = 0; i < car_vec_.size(); ++i) {
    records_vec_[i]->Reset();
  }
}

/*
_____________________________________________________________________________*/
