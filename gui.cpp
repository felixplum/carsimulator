#include "gui.h"
#include "ui_gui.h"

GUI::GUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GUI),
  sim_(0.1),
  DT_RECORDING_(0.1)
  {
    state_memory_.AddWriteRecord(sim_.AddNewCar(CT_BICYCLE));
    simulation_started_ = false;
    ui->setupUi(this);
    plot_window_ = new PlotWindow(parent);
    gui_timer_ = new QTimer(this);
    connect(gui_timer_, SIGNAL(timeout()), this, SLOT(GuiTimerUpdate()));
    scene_ = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene_);
  }

GUI::~GUI() {
  SetStatus(RS_STOPPED);
  delete ui;
}


void GUI::on_pushButton_startpause_clicked() {
  if (simulation_started_) {
    SetStatus(RS_STOPPED);
  } else {
    SetStatus(RS_RUNNING);
  }
}

// do plotting:
void GUI::on_pushButton_plot_clicked()
{
  plot_window_->show();
  std::vector<RecordPtr> record_vec = state_memory_.GetRecordPtrVec();
  plot_window_->InitPlot(record_vec);
}

void GUI::GuiTimerUpdate() {
  if (plot_window_->isVisible()) {
    plot_window_->Plot();
    plot_window_->update();
  }
  DrawSimulation();
}

void GUI::on_pushButton_reset_clicked()
{
  SetStatus(RS_STOPPED);
  // reset state
  state_memory_.ResetState();
  sim_.ResetState();
}

void GUI::SetStatus(RunState status) {
  if (status == RS_RUNNING) {
      simulation_started_ = true;
      ui->pushButton_startpause->setText("Stop");
      sim_.ChangeRunStatus(RS_RUNNING);
      ui->pushButton_startpause->setStyleSheet("background-color: blue");
      state_memory_.ToggleRecording(true, DT_RECORDING_);
      gui_timer_->start(DT_RECORDING_*1e3);
    } else if (status == RS_STOPPED) {
      simulation_started_ = false;
      ui->pushButton_startpause->setText( "Start");
      sim_.ChangeRunStatus(RS_STOPPED);
      ui->pushButton_startpause->setStyleSheet("");
      state_memory_.ToggleRecording(false);
      gui_timer_->stop();
    }
}

void GUI::DrawSimulation() {
  ui->graphicsView->setSceneRect(0, 0, 500, 500);
  QBrush blueBrush(Qt::blue);
  QPen outlinePen(Qt::black);
  outlinePen.setWidth(2);
  QGraphicsRectItem* rectangle = scene_->addRect(100, 0, 80, 100, outlinePen, blueBrush);
  //rectangle->set
      //drawBackground
}
