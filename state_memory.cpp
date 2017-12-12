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
    boost::this_thread::sleep_for(
          boost::chrono::milliseconds(static_cast<int>(dt_sample_*1e3)));
    //std::cout << "This thread is recording lolol" << std::endl;
    for (size_t i = 0; i < car_vec_.size(); ++i) {
      records_vec_[i]->AddState(car_vec_[i]->GetCarState().GetStateVector(),
                                car_vec_[i]->GetCarState().GetTime());
    }
  }
  std::cout << "recording stopped" << std::endl;
}
/*
_____________________________________________________________________________*/
void StateMemory::AddReadRecord(CarPtr car, const std::string& file_name) {
  std::cerr << "READFROM FILE method still missing!!" << std::endl;
  /* to: read tab delimited file [x y phi] [u_1 u_2]*/
  RecordPtr new_record(new Record(car->GetCarState().GetStateVector().size(),
                       Record::RecordType::RT_READ_FROM));
  records_vec_.push_back(std::move(new_record));
  car_vec_.push_back(car);
}
/*
_____________________________________________________________________________*/
void StateMemory::AddWriteRecord(CarPtr car) {
    RecordPtr new_record(new Record(car->GetCarState().GetStateVector().size(),
                         Record::RecordType::RT_WRITE_TO));
  records_vec_.push_back(std::move(new_record));
  car_vec_.push_back(car);
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

std::vector<RecordPtr> StateMemory::GetRecordPtrVec() const {
  return records_vec_;
}


/*
_____________________________________________________________________________*/
