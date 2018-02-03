#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGridLayout>
#include <QGraphicsRectItem>
#include <QListWidgetItem>
#include <plot_window.h>
#include <params_window.h>
#include "simulator.h"
#include "state_memory.h"
#include "car_model_bicycle.h"
#include "car_model_bicycle_dynamic.h"
#include <memory>
#include <QTimer>
#include "customtypes.h"
#include "map.h"
#include "udp_client.h"
#include <map>

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

  void on_pushButton_udp_clicked();

  void on_pushButton_addcar_clicked();

  void on_pushButton_remove_clicked();

  void on_horizontalSlider_valueChanged(int value);

  void on_pushButton_setcurrent_clicked();

  void on_pushButton_params_clicked();

  void on_pushButton_copycar_clicked();

  void on_pushButton_resettoudp_clicked();

  void on_carlistWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

private:
    void SetStatus(RunState status);
    void DrawSimulation();
    void TransformCarToViewCoord(Pose* car_pose) const;
    CarPtr AddCar(bool add_to_sim, std::string* id = NULL);
    bool RemoveCar(CarPtr car_ptr);
    void RunListener();
    //
    Ui::GUI *ui;
    PlotWindow *plot_window_;
    ParamsWindow* params_window_;
    QCustomPlot * custom_plot_;
    QTimer* gui_timer_;
    QGraphicsScene *scene_;
    QGraphicsScene *scene_egoview_;
    QImage *image_pov_;
    std::vector<QGraphicsRectItem*> car_rect_vec_;
    float pixel_per_meter_;
    QPixmap background_img_;
    //
    const float DT_RECORDING_;
    bool simulation_started_;
    Simulator sim_;
    std::shared_ptr<Map> map_;
    std::vector<CarPtr> cars_;
    std::map<CarPtr, QListWidgetItem*> carptr_listitem_map_;
    std::map<QListWidgetItem*, CarPtr> listitem_carptr_map_;
    CarPtr car_udp_;
    StateMemory state_memory_;
    UDPClient client_;
};

#endif // GUI_H
