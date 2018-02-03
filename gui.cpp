#include "gui.h"
#include "ui_gui.h"

GUI::GUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GUI),
  DT_RECORDING_(0.1),
  sim_(0.025) // simulation step size
  {
    // basic gui setup
    ui->setupUi(this);
    plot_window_ = new PlotWindow(&state_memory_, parent);
    params_window_ = new ParamsWindow(parent);
    scene_egoview_ = new QGraphicsScene(this); // scene for POV
    scene_ = new QGraphicsScene(this); // scene for map
    gui_timer_ = new QTimer(this); // timer for animation
    connect(gui_timer_, SIGNAL(timeout()), this, SLOT(GuiTimerUpdate()));
    // load background image / grid into Graphicsview_1
    if(!background_img_.load("grid_round_obstacle.bmp")) {
      std::cerr << "Couldn't load background grid" << std::endl;
      return;
    }
    // Scale image for display to fixed width, and scale graphicsview to fit image
    QPixmap background_img_scaled = background_img_.scaledToWidth(570);
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
    //
    ui->pushButton_resettoudp->setDisabled(true);
    ui->pushButton_setcurrent->setDisabled(true);
    // Init. simulator and add car
    std::unique_ptr<Map> map_tmp(new Map(background_img_));
    map_ = std::move(map_tmp);
    simulation_started_ = false;
    AddCar(true);
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
}

void GUI::GuiTimerUpdate() {
  static uint cnt = 0;
  cnt++;
  // update plot at half the timer rate
  if (plot_window_->isVisible() && ((cnt & 2) == 0)) {
    plot_window_->AddCurrentStateToGraph();
    plot_window_->update();
  }
  DrawSimulation();
}

void GUI::on_pushButton_reset_clicked()
{
  SetStatus(RS_STOPPED); // terminate thread
  // reset state
  state_memory_.ResetState();
  sim_.ResetState();
  DrawSimulation();
}

/* Thread loop; update state of car_udp_; client is blocking*/

void GUI::on_pushButton_udp_clicked() {
  static bool is_active = false;
  is_active = !is_active;
  if (is_active) {
    ui->pushButton_udp->setText("Disconnect");
    ui->pushButton_udp->setStyleSheet("background-color: green");
    std::string id = "(via UDP)";
    if (!car_udp_) {
      car_udp_ = AddCar(false, &id); // don't add to simulation
    }
    client_.ToogleCarUpdate(true, car_udp_);
    ui->pushButton_resettoudp->setDisabled(false);
  } else {
    ui->pushButton_udp->setText("Connect \n to hardware");
    ui->pushButton_udp->setStyleSheet("");
    client_.ToogleCarUpdate(false);
  }

}

void GUI::SetStatus(RunState status) {
  if (status == RS_RUNNING && !simulation_started_) {
      simulation_started_ = true;
      ui->pushButton_startpause->setText("Stop");
      sim_.ChangeRunStatus(RS_RUNNING);
      ui->pushButton_startpause->setStyleSheet("background-color: blue");
      state_memory_.ToggleRecording(true, DT_RECORDING_);
      gui_timer_->start(0.025*1e3); // 25 fps
      ui->horizontalSlider->setValue(ui->horizontalSlider->maximum());
      ui->pushButton_setcurrent->setDisabled(true);
    } else if (status == RS_STOPPED && simulation_started_) {
      simulation_started_ = false;
      ui->pushButton_startpause->setText( "Start");
      ui->pushButton_startpause->setStyleSheet("");
      sim_.ChangeRunStatus(RS_STOPPED);
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

CarPtr GUI::AddCar(bool add_to_sim, std::string* id) {
  // stop sim. if running:
  bool sim_started = simulation_started_;
  if (sim_started) SetStatus(RS_STOPPED);
  // add car
  CarPtr car(new CarModelBicycleDynamic(*map_));
  cars_.push_back(car);
  car->SetPovDimension(1., 1.);
  if (add_to_sim) sim_.AddCarPtr(car);
  state_memory_.AddWriteRecord(car);
  car_rect_vec_.push_back(scene_->addRect(0, 0, 0, 0));
  // add car to list
  std::string car_name("Car " + std::to_string(car_rect_vec_.size()));
  if (id) car_name += " " + *id;
  QListWidgetItem* listitem =
      new QListWidgetItem(QString::fromStdString(car_name));
  ui->carlistWidget->addItem(listitem);
  carptr_listitem_map_[car] = listitem;
  listitem_carptr_map_[listitem] = car;
  // update color:
  for (size_t i = 0; i < car_rect_vec_.size(); ++i) {
    int val = (255*i)/car_rect_vec_.size();
    QColor color(0, val, 255, 120);
    QBrush colorBrush(color);
    car_rect_vec_[i]->setBrush(colorBrush);
    ui->carlistWidget->item(i)->setBackgroundColor(color);
  }
  // continue sim.
  if (sim_started) SetStatus(RS_RUNNING);
  return car;
}

bool GUI::RemoveCar(CarPtr car_ptr) {
  bool found = false;
  bool sim_started = simulation_started_;
  if (sim_started) SetStatus(RS_STOPPED);
  for (size_t i = 0; i < cars_.size(); ++i) {
    if (car_ptr == cars_[i]) {
      cars_.erase(cars_.begin()+i);
      scene_->removeItem(car_rect_vec_[i]);
      car_rect_vec_.erase(car_rect_vec_.begin() + i);
      QListWidgetItem* item = carptr_listitem_map_[car_ptr];
      listitem_carptr_map_.erase(
            listitem_carptr_map_.find(carptr_listitem_map_[car_ptr]));
      carptr_listitem_map_.erase(carptr_listitem_map_.find(car_ptr));
      ui->carlistWidget->takeItem(ui->carlistWidget->row(item));
      if (car_ptr == car_udp_) {
        ui->pushButton_resettoudp->setDisabled(true);
        car_udp_.reset();
      }
      found = true;
      break;
    }
  }
  found = found & state_memory_.RemoveRecordPtr(car_ptr);
  found = found & sim_.RemoveCarPtr(car_ptr);
  if (sim_started) SetStatus(RS_RUNNING);
  return found;
}

/*
_________________________________________________________________________*/
void GUI::DrawSimulation() {

  pixel_per_meter_ = ui->graphicsView->width()/(CONSTANTS::X_MAX_METER);

  // Get current car state
  if (cars_.empty()) return;
  for (size_t i = 0; i < cars_.size(); ++i) {
    std::vector<float> state_vec = cars_[i]->GetCarState().GetStateVector();
//    float t_sim = cars_[i]->GetCarState().GetTime();
    if (state_vec.size() < 3 ) return;
    // Draw car rectangle:
    float width_m = cars_[i]->params_.width;
    float length_m = cars_[i]->params_.lf + cars_[i]->params_.lr;
    Pose car_pose(state_vec[0], state_vec[1], state_vec[2]);
    Pose car_pose_pixel = car_pose;
    TransformCarToViewCoord(&car_pose_pixel);
    float car_width_view = pixel_per_meter_ * width_m;
    float car_length_view = pixel_per_meter_* length_m;
    car_rect_vec_[i]->setRect(car_pose_pixel.x, car_pose_pixel.y-0.5*car_width_view,
                              car_length_view, car_width_view);

    car_rect_vec_[i]->setTransformOriginPoint(
                                car_rect_vec_[i]->boundingRect().center());
    car_rect_vec_[i]->setRotation((CONSTANTS::RAD2DEG)*car_pose_pixel.phi);
  }
  // update global map from graphicsview
//  map_->SetMap(ui->graphicsView->grab());


  // Draw POV of car
  CarPtr car_ptr = listitem_carptr_map_[ui->carlistWidget->currentItem()];
  if(!car_ptr) {
     car_ptr = cars_[0];
  }
  const QImage& image_ref = car_ptr->GetLocalGrid().
                            scaledToWidth(ui->graphicsView_2->width());
  QPixmap grid_pixmap = QPixmap::fromImage(image_ref);
  // get waypoints in grid (pixel) and car (m) coord.:
  std::vector<Point> waypoints_local_p;
  if (typeid(*car_ptr) == typeid(CarModelBicycle)) {
    CarModelBicycle* bm = dynamic_cast<CarModelBicycle*>(car_ptr.get());
    bm->GetWaypointsPixel(&waypoints_local_p);
  } else if (typeid(*car_ptr) == typeid(CarModelBicycleDynamic)) {
      CarModelBicycleDynamic* bm = dynamic_cast<CarModelBicycleDynamic*>
                                      (car_ptr.get());
      bm->GetWaypointsPixel(&waypoints_local_p);
  }
  // draw waypoints
  QPainter painter(&grid_pixmap);
  painter.setPen(Qt::red);
  for (size_t i = 0; i < waypoints_local_p.size(); ++i) {
    painter.drawEllipse(waypoints_local_p[i].y, waypoints_local_p[i].x,
                        10, 10);
  }
  // draw pixmap
  scene_egoview_->clear();
  scene_egoview_->addPixmap(grid_pixmap);
}


void GUI::on_pushButton_addcar_clicked()
{
    AddCar(true);
}

void GUI::on_pushButton_remove_clicked()
{
  QListWidgetItem* item = ui->carlistWidget->currentItem();
  if (!item) return;
  RemoveCar(listitem_carptr_map_[item]);
}

void GUI::on_horizontalSlider_valueChanged(int value)
{
  ui->pushButton_setcurrent->setDisabled(value == ui->horizontalSlider->maximum());
  CarPtr car_ptr = listitem_carptr_map_[ui->carlistWidget->currentItem()];
  // get record
  RecordPtr record_ptr = state_memory_.GetRecord(car_ptr);
  if (!record_ptr) return;
  // save current car state
  std::vector<float> state_vec_curr = car_ptr->GetCarState().GetStateVector();
  float t_curr = car_ptr->GetCarState().GetTime();
  // get state at idx of slider value (percent of max. index)
  int state_idx = (record_ptr->GetStateCount()-1)*
                  (float(value)/float(ui->horizontalSlider->maximum()));
  std::vector<float> state_vec;
  float t_sim;
  record_ptr->ReadStateAtIdx(&state_vec, &t_sim, state_idx);
  car_ptr->GetCarState().SetState(state_vec, t_sim);
  // update local map and controls (for waypoints) for display
  car_ptr->UpdateLocalMap();
  std::vector<float> u_out(2, 0.);
  car_ptr->GetControl(&u_out);
  DrawSimulation();
  // set car state back to original one
  car_ptr->GetCarState().SetState(state_vec_curr, t_curr);
}

void GUI::on_pushButton_setcurrent_clicked()
{
  CarPtr car_ptr = listitem_carptr_map_[ui->carlistWidget->currentItem()];
  // get record
  RecordPtr record_ptr = state_memory_.GetRecord(car_ptr);
  if (!record_ptr) return;

  // get state at idx of slider value (percent of max. index)
  int state_idx = (record_ptr->GetStateCount()-1)*
                  (float(ui->horizontalSlider->value())/
                   float(ui->horizontalSlider->maximum()));
  std::vector<float> state_vec;
  float t_sim;
  record_ptr->ReadStateAtIdx(&state_vec, &t_sim, state_idx);
  car_ptr->GetCarState().SetState(state_vec, t_sim);
  // erase state memory
  record_ptr->ResetToIdx(state_idx);
  ui->horizontalSlider->setValue(ui->horizontalSlider->maximum());
}

void GUI::on_pushButton_params_clicked()
{
  CarPtr car_ptr = listitem_carptr_map_[ui->carlistWidget->currentItem()];
  if (!car_ptr) return;
  params_window_->SetActiveParams(&car_ptr->params_);
  params_window_->show();
}

void GUI::on_pushButton_copycar_clicked()
{
  CarPtr car_ptr = listitem_carptr_map_[ui->carlistWidget->currentItem()];
  if (!car_ptr) return;
  const std::vector<float>& current_state = car_ptr->GetCarState().GetStateVector();
  std::string id = "(copy)";
  CarPtr car_new = AddCar(true, &id);
  car_new->GetCarState().SetState(current_state, 0.);
}

void GUI::on_pushButton_resettoudp_clicked()
{
  CarPtr car_ptr = listitem_carptr_map_[ui->carlistWidget->currentItem()];
  if (!car_ptr || car_ptr == car_udp_) return;
  RecordPtr record_ptr = state_memory_.GetRecord(car_ptr);
  const std::vector<float>& state_vec = car_udp_->GetCarState().GetStateVector();
  record_ptr->Reset();
  car_ptr->GetCarState().SetState(state_vec, 0);
  DrawSimulation();
}

void GUI::on_carlistWidget_currentItemChanged(QListWidgetItem *current,
                                              QListWidgetItem *previous)
{
  CarPtr car_ptr = listitem_carptr_map_[current];
  if(!car_ptr) return;
  plot_window_->SetActiveCar(car_ptr);
}
