#include "plot_window.h"

PlotWindow::PlotWindow(QWidget *parent) : QWidget(parent) {

  custom_plot_ = new QCustomPlot(this);
  layout_ = new QGridLayout(this);
  layout_->addWidget(custom_plot_, 0, 0);
  //plot_window_->setLayout(layout);
  custom_plot_->addGraph();
  custom_plot_->graph(0)->setLineStyle(QCPGraph::lsLine);
  custom_plot_->addGraph();
  custom_plot_->graph(1)->setLineStyle(QCPGraph::lsLine);
  this->move(300, 300);
  this->resize(200, 200);
  record_vec_.resize(0);
}


void PlotWindow::InitPlot(std::vector<RecordPtr> record_vec) {
  record_vec_ = record_vec;
}

void PlotWindow::Plot() {

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
}

