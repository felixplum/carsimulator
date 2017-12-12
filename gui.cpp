#include "gui.h"
#include "ui_gui.h"

GUI::GUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GUI),
  sim_(0.1)
  {
    state_memory_.AddWriteRecord(sim_.AddNewCar(CT_BICYCLE));
    simulation_started_ = false;
    ui->setupUi(this);
    plot_window_ = new PlotWindow(parent);
    gui_timer_ = new QTimer(this);
    connect(gui_timer_, SIGNAL(timeout()), this, SLOT(GuiTimerUpdate()));
  }

GUI::~GUI() {
  delete ui;
}


void GUI::on_pushButton_startpause_clicked() {
  if (simulation_started_) {
      simulation_started_ = false;
      ui->pushButton_startpause->setText( "Start");
      sim_.ChangeRunStatus(RS_STOPPED);
      ui->pushButton_startpause->setStyleSheet("");
      state_memory_.ToggleRecording(false, 0.1);
      gui_timer_->stop();
  } else {
      simulation_started_ = true;
      ui->pushButton_startpause->setText("Stop");
      sim_.ChangeRunStatus(RS_RUNNING);
      ui->pushButton_startpause->setStyleSheet("background-color: blue");
      state_memory_.ToggleRecording(true, 0.1);
      gui_timer_->start(100);
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
}
