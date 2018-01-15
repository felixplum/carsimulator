#include "gui.h"
#include "ui_gui.h"

GUI::GUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GUI),
  DT_RECORDING_(0.04)
  {
    ui->setupUi(this);
    plot_window_ = new PlotWindow(parent);
    gui_timer_ = new QTimer(this);
    connect(gui_timer_, SIGNAL(timeout()), this, SLOT(GuiTimerUpdate()));
    scene_ = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene_);
    car_rect_vec_.push_back(scene_->addRect(0, 0, 0, 0));
    QBrush blueBrush(QColor(0, 100, 255, 120));
    car_rect_vec_[0]->setBrush(blueBrush);
    ui->graphicsView->setRenderHints(QPainter::Antialiasing);

    // load background image / grid into Graphicsview_1
    QPixmap background_img;
    if(!background_img.load("grid.bmp")) {
      std::cerr << "Couldn't load background grid" << std::endl;
      return;
    }
    QPixmap background_img_scaled = background_img.scaledToWidth(550);
    ui->graphicsView->resize(background_img_scaled.width(), background_img_scaled.height());
    scene_->setBackgroundBrush(QBrush(background_img_scaled));
    ui->graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    ui->graphicsView->setSceneRect(0, 0, ui->graphicsView->width(), ui->graphicsView->height());
    ui->graphicsView->setFrameShape(QGraphicsView::NoFrame);
    // Graphicsview_2 for ego-perspective of car
    int egoview_width = 200;
    int egoview_height= 200;
    ui->graphicsView_2->resize(egoview_width, egoview_height);
    scene_egoview_ = new QGraphicsScene(this);
    ui->graphicsView_2->setScene(scene_egoview_);
    ui->graphicsView_2->setSceneRect(0, 0, ui->graphicsView_2->width(), ui->graphicsView_2->height());
    ui->graphicsView_2->setFrameShape(QGraphicsView::NoFrame);
    // Init. simulator
    std::unique_ptr<Simulator> sim_tmp(new Simulator(0.04, background_img));
    sim_ = std::move(sim_tmp);
    state_memory_.AddWriteRecord(sim_->AddNewCar(CT_BICYCLE));
    simulation_started_ = false;
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
  SetStatus(RS_STOPPED); // terminate thread
  // reset state
  state_memory_.ResetState();
  sim_->ResetState();
}

//void GUI::closeEvent(QCloseEvent *bar) {
//  std::cout << "cloed1";
//}

void GUI::SetStatus(RunState status) {
  if (status == RS_RUNNING) {
      simulation_started_ = true;
      ui->pushButton_startpause->setText("Stop");
      sim_->ChangeRunStatus(RS_RUNNING);
      ui->pushButton_startpause->setStyleSheet("background-color: blue");
      state_memory_.ToggleRecording(true, DT_RECORDING_);
      gui_timer_->start(DT_RECORDING_*1e3);
    } else if (status == RS_STOPPED) {
      simulation_started_ = false;
      ui->pushButton_startpause->setText( "Start");
      sim_->ChangeRunStatus(RS_STOPPED);
      ui->pushButton_startpause->setStyleSheet("");
      state_memory_.ToggleRecording(false);
      gui_timer_->stop();
    }
}

/*
_________________________________________________________________________*/
void GUI::TransformCarToViewCoord(Pose* car_pose) const {
  /* QT coord: x pointing right, y to the bottom; origin top left corner */
  car_pose->y = ui->graphicsView->height()-(car_pose->y)*pixel_per_meter_;
  car_pose->x = (car_pose->x)*pixel_per_meter_;
  car_pose->phi *= -1.;
}
/*
_________________________________________________________________________*/
void GUI::DrawSimulation() {

  pixel_per_meter_ = ui->graphicsView->width()/(CONSTANTS::X_MAX_METER);
//  ui->graphicsView->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
//  QPen outlinePen(Qt::black);
//  outlinePen.setWidth(2);
  std::vector<RecordPtr> record_vec = state_memory_.GetRecordPtrVec();
  std::vector<float> state_vec;
  float t_sim;
  record_vec[0]->ReadLastState(&state_vec, &t_sim);
  if (state_vec.empty()) {
      std::cout <<"state empty :(" << std::endl;
      return;
   }
//  Pose car_pose(state_vec[0], state_vec[1], state_vec[2]);
  Pose car_pose(state_vec[0], state_vec[1], state_vec[2]);
  Pose car_pose_pixel = car_pose;
//  int val = map_.GetGridValueAtMetricPoint(Point(car_pose.x, car_pose.y));
//  std::cout << val << std::endl;
  TransformCarToViewCoord(&car_pose_pixel);
  float car_width_view = pixel_per_meter_ * (CONSTANTS::CAR_WIDTH_METER);
  float car_length_view = pixel_per_meter_* (CONSTANTS::CAR_LENGTH_METER);
  car_rect_vec_[0]->setRect(car_pose_pixel.x, car_pose_pixel.y-0.5*car_width_view,
                            car_length_view, car_width_view);

  car_rect_vec_[0]->setTransformOriginPoint(car_rect_vec_[0]->boundingRect().center());
  car_rect_vec_[0]->setRotation((CONSTANTS::RAD2DEG)*car_pose_pixel.phi);

  // Draw POV of car
  const QImage &image_ref = sim_->GetLocalGrid()
                            .scaledToWidth(ui->graphicsView_2->width());
  QPixmap grid_pixmap = QPixmap::fromImage(image_ref);
  // get waypoints in grid (pixel) and car (m) coord.:
  std::vector<Point> waypoints_local_p;
  std::vector<Point> waypoints_local_m;
  planner_.GetWaypoints(image_ref, &waypoints_local_p,
                        &waypoints_local_m, &pixel_per_meter_);
//  // draw waypoints
  QPainter painter(&grid_pixmap);
  painter.setPen(Qt::red);
  for (uint i = 0; i < waypoints_local_p.size(); ++i) {
    painter.drawEllipse(waypoints_local_p[i].y, waypoints_local_p[i].x, 10, 10);
  }
  // draw pixmap
  scene_egoview_->addPixmap(grid_pixmap);
}


