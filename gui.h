#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <plot_window.h>
#include "simulator.h"
#include "state_memory.h"
#include "car_model_bicycle.h"
#include <memory>
#include <QTimer>
#include "customtypes.h"
#include "map.h"
#include "pathplanner.h"

namespace Ui {
class GUI;
}

class GUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit GUI(QWidget *parent = 0);
    ~GUI();
//    void closeEvent(QCloseEvent *bar);

private slots:

  void on_pushButton_startpause_clicked();

  void on_pushButton_plot_clicked();
  void GuiTimerUpdate();

  void on_pushButton_reset_clicked();

private:
    void SetStatus(RunState status);
    void DrawSimulation();
    void TransformCarToViewCoord(Pose* car_pose) const;
    //
    Ui::GUI *ui;
    PlotWindow *plot_window_;
    QCustomPlot * custom_plot_;
    QTimer* gui_timer_;
    QGraphicsScene *scene_;
    QGraphicsScene *scene_egoview_;
    QImage *image_pov_;
    std::vector<QGraphicsRectItem*> car_rect_vec_;
    float pixel_per_meter_;
    //
    bool simulation_started_;
    Simulator sim_;
    Map map_;
    PathPlanner planner_;
    StateMemory state_memory_;
    const float DT_RECORDING_;
};

#endif // GUI_H
