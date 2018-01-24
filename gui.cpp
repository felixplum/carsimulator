#include "gui.h"
#include "ui_gui.h"

GUI::GUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GUI),
  DT_RECORDING_(0.04),
  listen_to_udp_(false)
  {
    // basic gui setup
    ui->setupUi(this);
    plot_window_ = new PlotWindow(parent);
    scene_egoview_ = new QGraphicsScene(this); // scenen for POV
    scene_ = new QGraphicsScene(this); // scene for map
    gui_timer_ = new QTimer(this); // timer for animation
    connect(gui_timer_, SIGNAL(timeout()), this, SLOT(GuiTimerUpdate()));

    // load background image / grid into Graphicsview_1
    QPixmap background_img;
    if(!background_img.load("grid_round_obstacle.bmp")) {
      std::cerr << "Couldn't load background grid" << std::endl;
      return;
    }
    // Scale image for display to fixed width, and scale graphicsview to fit image
    QPixmap background_img_scaled = background_img.scaledToWidth(550);
    ui->graphicsView->setScene(scene_);
    ui->graphicsView->resize(background_img_scaled.width(),
                             background_img_scaled.height());
    scene_->setBackgroundBrush(QBrush(background_img_scaled));
    ui->graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    ui->graphicsView->setSceneRect(0, 0, ui->graphicsView->width(),
                                   ui->graphicsView->height());
    ui->graphicsView->setFrameShape(QGraphicsView::NoFrame);
    ui->graphicsView->setRenderHints(QPainter::Antialiasing);
    // Graphicsview_2 for ego-perspective of car
    int egoview_width = 200;
    int egoview_height= 200;
    ui->graphicsView_2->setScene(scene_egoview_);
    ui->graphicsView_2->resize(egoview_width, egoview_height);
    ui->graphicsView_2->setSceneRect(0, 0, ui->graphicsView_2->width(),
                                     ui->graphicsView_2->height());
    ui->graphicsView_2->setFrameShape(QGraphicsView::NoFrame);
    // Init. simulator and add car
    std::unique_ptr<Simulator> sim_tmp(new Simulator(0.04, background_img));
    sim_ = std::move(sim_tmp);
    AddCar();
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

void GUI::RunListener() {
  UDPClient client(car_udp_, listen_to_udp_);
}


void GUI::on_pushButton_udp_clicked() {
  static bool is_active = false;
  is_active = !is_active;
  listen_to_udp_ = is_active;
  if (is_active) {
    ui->pushButton_udp->setText("Disconnect");
    ui->pushButton_udp->setStyleSheet("background-color: green");
    CarPtr car_udp = sim_->AddNewCar(CT_BICYCLE);
    car_udp_ = std::move(car_udp);
    state_memory_.AddWriteRecord(car_udp_);
    boost::thread t(&GUI::RunListener, this);
  } else {
    ui->pushButton_udp->setText("Connect \n to hardware");
    ui->pushButton_udp->setStyleSheet("");
//    SetStatus(RS_STOPPED);
    sim_->RemoveCarAtIdx(1);
    state_memory_.RemoveRecordAtIdx(1);
//    std::cout << "del. record" << std::endl;
  }

}

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

void GUI::AddCar() {
  car_rect_vec_.push_back(scene_->addRect(0, 0, 0, 0));
  QBrush blueBrush(QColor(0, 100, 255, 120));
  car_rect_vec_[0]->setBrush(blueBrush);
  state_memory_.AddWriteRecord(sim_->AddNewCar(CT_BICYCLE));
}

/*
_________________________________________________________________________*/
void GUI::DrawSimulation() {

  pixel_per_meter_ = ui->graphicsView->width()/(CONSTANTS::X_MAX_METER);


  // Get last car state from state memory:
  std::vector<RecordPtr> record_vec = state_memory_.GetRecordPtrVec();
  std::vector<float> state_vec;
  float t_sim;
  if (record_vec.empty() || !record_vec[0]) return;
  record_vec[0]->ReadLastState(&state_vec, &t_sim);
  if (state_vec.empty()) {
      std::cout <<"state empty :(" << std::endl;
      return;
   }
  // Draw car rectangle:
  Pose car_pose(state_vec[0], state_vec[1], state_vec[2]);
  Pose car_pose_pixel = car_pose;
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
  sim_->GetWaypointsPixel(&waypoints_local_p);
  // draw waypoints
  QPainter painter(&grid_pixmap);
  painter.setPen(Qt::red);
  for (size_t i = 0; i < waypoints_local_p.size(); ++i) {
    painter.drawEllipse(waypoints_local_p[i].y, waypoints_local_p[i].x, 10, 10);
  }
  // draw pixmap
  scene_egoview_->addPixmap(grid_pixmap);
}

