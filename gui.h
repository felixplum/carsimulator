#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <plot_window.h>
#include "simulator.h"
#include "state_memory.h"
#include "car_model_bicycle.h"
#include <memory>
#include <QTimer>

namespace Ui {
class GUI;
}

class GUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit GUI(QWidget *parent = 0);
    ~GUI();

private slots:

  void on_pushButton_startpause_clicked();

  void on_pushButton_plot_clicked();
  void GuiTimerUpdate();

  void on_pushButton_reset_clicked();

private:
    void SetStatus(RunState status);
    void DrawSimulation();
    //
    Ui::GUI *ui;
    PlotWindow *plot_window_;
    QCustomPlot * custom_plot_;
    QTimer* gui_timer_;
    QGraphicsScene *scene_;
    //
    bool simulation_started_;
    Simulator sim_;
    StateMemory state_memory_;
    const float DT_RECORDING_;
};

#endif // GUI_H
