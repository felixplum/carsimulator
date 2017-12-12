#include "gui.h"
#include "ui_gui.h"

GUI::GUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GUI)
  {
    state_memory_.AddWriteRecord(sim_.AddNewCar(CT_BICYCLE));
    simulation_started_ = false;
    ui->setupUi(this);
    plot_window_ = new PlotWindow(parent);
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
  } else {
      simulation_started_ = true;
      ui->pushButton_startpause->setText("Stop");
      sim_.ChangeRunStatus(RS_RUNNING);
      ui->pushButton_startpause->setStyleSheet("background-color: blue");
      state_memory_.ToggleRecording(true, 0.1);
  }
}

// do plotting:
void GUI::on_pushButton_plot_clicked()
{
  plot_window_->show();
  std::vector<RecordPtr> record_vec = state_memory_.GetRecordPtrVec();
  plot_window_->StartPlotting(record_vec);
}
