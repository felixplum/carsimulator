#include "plot_window.h"

PlotWindow::PlotWindow(StateMemory* state_memory,
                       QWidget *parent) : QWidget(parent) {
  car_current_ = NULL;
  state_memory_ = state_memory;
  do_rebuild_layout_ = false;
  layout_ = new QGridLayout(this);

  this->move(300, 300);
  this->resize(300, 700);
}

void PlotWindow::RebuildLayout() {
  if (!car_current_) return;
  for (auto &it : plots_) {
    delete it;
  }
  plots_.clear();

  const std::vector<std::string>& state_names =
      car_current_->GetCarState().GetStateNames();

  for (size_t i = 0; i < state_names.size(); ++i) {
    plots_.push_back(new QCustomPlot(this));
    layout_->addWidget(plots_[i], i, 0);
    plots_[i]->addGraph(0);
    plots_[i]->graph(0)->setLineStyle(QCPGraph::lsLine);
    plots_[i]->yAxis->setLabel(QString::fromStdString(state_names[i]));
    plots_[i]->yAxis->setNumberPrecision(1);
    if (i == state_names.size()-1) {
        plots_[i]->xAxis->setLabel("t [s]");
        plots_[i]->xAxis->setNumberPrecision(1);
    }
  }
  PlotHistory();
}

void PlotWindow::SetActiveCar(CarPtr car_ptr) {
  if (car_current_ != car_ptr) {
    car_current_ = car_ptr;
    do_rebuild_layout_ = true;
  } else {do_rebuild_layout_ = false;}
}

void PlotWindow::PlotHistory() {
  if (!car_current_) return;
  RecordPtr record = state_memory_->GetRecord(car_current_);
  size_t skip_val = (record->GetStateCount())/100 + 1;
  for (size_t i = 0; i < record->GetStateCount(); i+=skip_val) {
    std::vector<float> state;
    float time;
    record->ReadStateAtIdx(&state, &time, i);
    for (size_t i = 0; i < plots_.size(); ++i) {
      // add state; map angle to 0..2*PI
      if (car_current_->GetCarState().GetStateNames()[i] == "phi") {
        plots_[i]->graph(0)->addData(time, fmod(state[i], 2*3.141));
      } else {
        plots_[i]->graph(0)->addData(time, state[i]);
      }
      plots_[i]->graph(0)->rescaleAxes();
      plots_[i]->replot();
    }
  }
}

void PlotWindow::AddCurrentStateToGraph() {
  if (!car_current_) return;
  if (do_rebuild_layout_) {
    RebuildLayout();
    do_rebuild_layout_ = false;
  }
  const std::vector<float>& state = car_current_->GetCarState().GetStateVector();
  float time =  car_current_->GetCarState().GetTime();
  for (size_t i = 0; i < plots_.size(); ++i) {
    // add state; map angle to 0..2*PI
    if (car_current_->GetCarState().GetStateNames()[i] == "phi") {
      plots_[i]->graph(0)->addData(time, fmod(state[i], 2*3.141));
    } else {
      plots_[i]->graph(0)->addData(time, state[i]);
    }
    plots_[i]->graph(0)->rescaleAxes();
    plots_[i]->replot();
  }
/*
  std::vector<float> state_vec_return;
  float time_return;
  if (record_vec_.empty()) return;
  int no_states = record_vec_[0]->GetStateCount();
  if (no_states == 0) return;
  //std::cout << "no states is " << no_states << std::endl;
  QVector<double> xx;
  QVector<double> yy;
  QVector<double> time_vec;
  float max_val = -1e9;
  for (int i = 0; i < no_states; ++i) {
    record_vec_[0]->ReadStateAtIdx(&state_vec_return, &time_return, i);
    xx.push_back(state_vec_return[0]);
    yy.push_back(state_vec_return[1]);
    time_vec.push_back(time_return);
    //std::cout << "x: " << xx.back() << " t: " << time_vec.back() << std::endl;
    if (fabs(xx.back()) > max_val) max_val = fabs(xx.back());
    if (fabs(yy.back()) > max_val) max_val = fabs(yy.back());
  }

  custom_plot_->graph(0)->setData(time_vec, xx, true);
  custom_plot_->graph(1)->setData(time_vec, yy, true);
  custom_plot_->xAxis->setRange(0, time_vec.back());
  custom_plot_->yAxis->setRange(-max_val, max_val);
//  custom_plot_->graph(0)->setPen(QPen(QColor(255, 100, 0)));
//  custom_plot_->graph(0)->setLineStyle(QCPGraph::lsLine);
//  custom_plot_->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
  custom_plot_->replot();
  //custom_plot_->show();
  */
}

