#ifndef PLOT_WINDOW_H
#define PLOT_WINDOW_H

#include <QWidget>
#include "../../Libs/qcustomplot/qcustomplot.h"
#include "state_memory.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>


class PlotWindow : public QWidget
{
  Q_OBJECT
public:
  explicit PlotWindow(QWidget *parent = nullptr);
  void Plot();
  void StartPlotting(std::vector<RecordPtr> record_vec);

signals:

public slots:

private:
  QCustomPlot* custom_plot_;
  QGridLayout *layout_;
  std::vector<RecordPtr> record_vec_;
  virtual void paintEvent(QPaintEvent * event);
};

#endif // PLOT_WINDOW_H
