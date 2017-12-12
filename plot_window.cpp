#include "plot_window.h"

PlotWindow::PlotWindow(QWidget *parent) : QWidget(parent) {

  custom_plot_ = new QCustomPlot(this);
  layout_ = new QGridLayout(this);
  layout_->addWidget(custom_plot_, 0, 0);
  //plot_window_->setLayout(layout);
  custom_plot_->addGraph();
  custom_plot_->graph(0)->setLineStyle(QCPGraph::lsLine);
  this->move(300, 300);
  this->resize(200, 200);
  record_vec_.resize(0);
}


void PlotWindow::StartPlotting(std::vector<RecordPtr> record_vec) {
  record_vec_ = record_vec;
//  boost::thread t(&PlotWindow::Run, this);
  Plot();
}
void PlotWindow::Plot() {

  std::vector<float> state_vec_return;
  float time_return;
  if (record_vec_.empty()) return;
  int no_states = record_vec_[0]->GetStateCount();
  QVector<double> xx;
  QVector<double> yy;
  QVector<double> time_vec;
  float max_val = -1e9;
  for (int i = 0; i < no_states; ++i) {
    record_vec_[0]->ReadStateAtIdx(&state_vec_return, &time_return, i);
    xx.push_back(state_vec_return[0]);
    yy.push_back(state_vec_return[1]);
    time_vec.push_back(time_return);
    if (fabs(xx.back()) > max_val) max_val = fabs(xx.back());
    if (fabs(yy.back()) > max_val) max_val = fabs(yy.back());
  }
//  record_vec[0]->ReadLastState(&state_vec_return, &time_return);
//  printf("last x|y|phi|t: %f %f %f %f", state_vec_return[0], state_vec_return[1],
//                                        state_vec_return[2], time_return);

//  std::vector<double> x(100);
//  std::vector<double> y(x.size());
//  for (size_t i = 0; i < x.size(); ++i) {
//    x[i] = i*0.01;
//    y[i] = x[i]*x[i];
//  }
//  QVector<double> xx = QVector<double>::fromStdVector(x);
//  QVector<double> yy = QVector<double>::fromStdVector(y);
  custom_plot_->graph(0)->setData(xx,yy, true);
  custom_plot_->xAxis->setRange(-max_val, max_val);
  custom_plot_->yAxis->setRange(-max_val, max_val);
  custom_plot_->replot();
  //custom_plot_->show();
}

