#ifndef STATE_MEMORY_H
#define STATE_MEMORY_H

#include<car.h>
#include<simulator.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <memory> // for smart ptr
#include <QWidget>
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
  boost::mutex rw_lock_;
 public:
  Record(int state_size, RecordType record_type){
    record_type_ = record_type;
    state_size_ = state_size;
    state_count_ = 0;
    // allocate memory for 1min@10Hz storage
    state_storage_.reserve(state_size_*10*60);
    time_vec_.reserve(state_size_*10*60);
    //  std::cout << "Added new record with state_storage size " <<
    //  state_storage_.size() << std::endl;
  }
  void AddState(const std::vector<float>& state_vec, float time) {
    boost::mutex::scoped_lock(rw_lock_);
    state_storage_.insert(state_storage_.end(), state_vec.begin(),
                          state_vec.end());
    time_vec_.push_back(time);
    state_count_++;
  }
  bool ReadLastState(std::vector<float>* state_vec_return,
                     float* time_return) const {
    if (state_count_ == 0) {return false;}
    return ReadStateAtIdx(state_vec_return, time_return, state_count_-1);
  }
  bool ReadStateAtIdx(std::vector<float>* state_vec_return,
                      float* time_return, int state_idx) const {
    //std::cout << "entering" <<std::endl;
    boost::mutex::scoped_lock(rw_lock_);
    if (state_size_ == 0 || state_idx > state_count_-1) {return false;}
    std::vector<float> ret_vec(
          state_storage_.begin()+state_idx*state_size_,
          state_storage_.begin()+state_idx*state_size_+state_size_-1);
    *state_vec_return = std::move(ret_vec);
    *time_return = time_vec_.back();
    std::cout << "Record, return t = " <<*time_return<< std::endl;
    std::cout << "Record, return x = " <<(*state_vec_return)[0]<< std::endl;
    return true;
  }
  int GetStateCount() const {
    return state_count_;
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
  void ToggleRecording(bool activate_recording, float dt_sample);
  std::vector<RecordPtr> GetRecordPtrVec() const;

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
