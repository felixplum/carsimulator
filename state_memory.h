#ifndef STATE_MEMORY_H
#define STATE_MEMORY_H

#include<car.h>
#include<simulator.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <memory> // for smart ptr
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>
/* This class provides methods for storing and accessing states
   #Usage:
    - Passing ptr to car,  */

/* For each simulation object, a record will be created
   _______________________________________________________________ */
struct Record {
  enum RecordType {RT_READ_FROM, RT_WRITE_TO};
 private:
  int state_size_;
  int state_count_;
  std::vector<float> time_vec_;
  RecordType record_type_;
  // stores state vecs. sequentially for faster access
  std::vector<float> state_storage_;
 public:
  Record(int state_size, RecordType record_type){
    record_type_ = record_type;
    state_size_ = state_size;
    // allocate memory for 1min@10Hz storage
    state_storage_.reserve(state_size_*10*60);
    time_vec_.reserve(state_size_*10*60);
  }
  void AddState(const std::vector<float>& state_vec, float time) {
    state_storage_.insert(state_storage_.end(), state_vec.begin(),
                          state_vec.end());
    time_vec_.push_back(time);
  }
  bool ReadLastState(std::vector<float>* state_vec_return,
                     float* time_return) const {
    if (state_count_ == 0) {return false;}
    return ReadStateAtIdx(state_vec_return, time_return, state_count_-1);
  }
  bool ReadStateAtIdx(std::vector<float>* state_vec_return,
                      float* time_return, int state_idx) const {
    if (state_size_ == 0 || state_idx > state_count_-1) {return false;}
    std::vector<float> ret_vec(
          state_storage_.begin()+state_idx*state_size_,
          state_storage_.begin()+state_idx*state_size_+state_size_-1);
    *state_vec_return = std::move(ret_vec);
    *time_return = time_vec_.back();
    return true;
  }
};

typedef std::unique_ptr<Record> RecordPtr;

/* Main declaration
___________________________________________________________________*/
class StateMemory {
 public:
  StateMemory();
  void AddReadRecord(CarPtr car, const std::string& file_name);
  void AddWriteRecord(CarPtr car);
  void ToggleRecording(bool activate_recording, float dt_sample);

 private:
  // main thread loop
  void RunRecording();
  void WriteToFile(const Record& record, CarPtr car,  // todo: not implemented
                   std::string file_name);
  // store pointers for efficient memory access
  std::vector<RecordPtr> records_vec_;
  std::vector<CarPtr> car_vec_;
  bool is_recording_;
  float dt_sample_;

};

#endif // STATE_MEMORY_H
