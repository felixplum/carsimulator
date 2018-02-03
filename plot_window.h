#ifndef PLOT_WINDOW_H
#define PLOT_WINDOW_H

#include <QWidget>
#include "qcustomplot/qcustomplot.h"
#include "state_memory.h"
#include <QGridLayout>
#include <car.h>


class PlotWindow : public QWidget
{
  Q_OBJECT
public:
  explicit PlotWindow(StateMemory* state_memory, QWidget *parent = nullptr);
  void AddCurrentStateToGraph();
  void SetActiveCar(CarPtr car_ptr);

signals:

public slots:

private:
  QGridLayout *layout_;
  StateMemory* state_memory_;
  std::vector<QCustomPlot*> plots_;
  CarPtr car_current_;
  bool do_rebuild_layout_;
  //
  void RebuildLayout();
  void PlotHistory();
};

#endif // PLOT_WINDOW_H
