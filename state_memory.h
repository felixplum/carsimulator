#ifndef STATE_MEMORY_H
#define STATE_MEMORY_H

#include<car.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <memory> // for smart ptr
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>
#include <QWidget>
#include <chrono>
#include <thread>
/* This class provides methods for storing and accessing states
   #Usage:
    - Passing ptr to car,  */

/* For each simulation object, a record will be created
   _______________________________________________________________ */
struct Record {
  enum RecordType {RT_READ_FROM, RT_WRITE_TO};
 private:
  size_t state_size_;
  size_t state_count_;
  std::vector<float> time_vec_;
  // stores state vecs. sequentially for faster access
  std::vector<float> state_storage_;
  boost::mutex rw_lock_;
 public:
  RecordType record_type_;
  Record(int state_size, RecordType record_type){
    record_type_ = record_type;
    state_size_ = state_size;
    Reset();
  }
  void AddState(const std::vector<float>& state_vec, float time) {
    boost::mutex::scoped_lock(rw_lock_);
    state_storage_.insert(state_storage_.end(), state_vec.begin(),
                          state_vec.end());
    time_vec_.push_back(time);
    state_count_++;
  }
  void ResetToIdx(size_t idx) {
    boost::mutex::scoped_lock(rw_lock_);
    if (state_size_ == 0 || idx > state_count_-1) {return;}
    std::vector<float> tmp_state (state_storage_.begin(),
          state_storage_.begin()+idx*state_size_+state_size_);
    state_storage_ = tmp_state;
    std::vector<float> tmp_time(time_vec_.begin(), time_vec_.begin()+idx+1);
    time_vec_ = tmp_time;
    state_count_ = time_vec_.size();
  }
  bool ReadLastState(std::vector<float>* state_vec_return,
                     float* time_return) const {
    if (state_count_ == 0) {return false;}
    return ReadStateAtIdx(state_vec_return, time_return, state_count_-1);
  }
  bool ReadStateAtIdx(std::vector<float>* state_vec_return,
                      float* time_return, size_t state_idx) const {
    //std::cout << "ReadStateAtIdx " << state_idx<<std::endl;
    boost::mutex::scoped_lock(rw_lock_);
    if (state_count_ == 0 || state_idx > state_count_-1) {return false;}
    std::vector<float> ret_vec(
          state_storage_.begin()+state_idx*state_size_,
          state_storage_.begin()+state_idx*state_size_+state_size_);
//    std::cout << "read range " << state_idx*state_size_ << " to " << state_idx*state_size_+state_size_-1 << std::endl;
//    std::cout << "phi rad, ri = " << ret_vec[2] << std::endl;
    *state_vec_return = std::move(ret_vec);
    *time_return = time_vec_[state_idx];
//    std::cout << "Record, return t = " <<*time_return <<
//                 " return x = " <<(*state_vec_return)[0]<< std::endl;
    return true;
  }
  size_t GetStateCount() const {
    return state_count_;
  }
  void Reset() {
    state_count_ = 0;
    // allocate memory for 1min@10Hz storage
    state_storage_.clear();
    time_vec_.clear();
    state_storage_.reserve(state_size_*10*60);
    time_vec_.reserve(state_size_*10*60);
  }
  void PrintState() {
    std::cout << "State size: " << state_size_ << " State count: " <<
                 state_count_ << std::endl;
    std::cout << "Size of state storage: " << state_storage_.size() <<
                 std::endl;
  }
};

typedef std::shared_ptr<Record> RecordPtr;

/* Main declaration
___________________________________________________________________*/
class StateMemory : QWidget {
 public:
  StateMemory();
//  void SetCallback(std::function<void()> callback);
  void AddReadRecord(CarPtr car, const std::string& file_name);
  void AddWriteRecord(CarPtr car);
  bool RemoveRecordPtr(CarPtr car_ptr);
  void ToggleRecording(bool activate_recording, float dt_sample = 0.1);
  std::vector<RecordPtr> GetRecordPtrVec() const;
  RecordPtr GetRecord(CarPtr car_ptr) const;
  void ResetState();

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
  std::unique_ptr<std::thread> thread_;

};

#endif // STATE_MEMORY_H
