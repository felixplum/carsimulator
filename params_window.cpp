#include "params_window.h"

ParamsWindow::ParamsWindow(QWidget *parent) : QWidget(parent)
{
  layout_ = new QGridLayout(this);
  CarParams params_protoype;
  this->move(300, 300);
  this->resize(200, 200);
  // generate layout from general parameter description:
  // It's possible that not all params are used by specific car
  for (size_t i = 0; i < params_protoype.description.size(); ++i) {
    input_fields_.push_back(new QLineEdit(this));
    labels_.push_back(new QLabel(this));
    labels_[i]->setText(QString::fromStdString(
                          params_protoype.description[i]));
    input_fields_[i]->setText(QString::number(*params_protoype.value_ptrs[i]));
    QDoubleValidator* validator = new QDoubleValidator(0.0, 100.0, 5, input_fields_[i]);
    validator->setNotation(QDoubleValidator::StandardNotation);
    input_fields_[i]->setValidator(validator);
    layout_->addWidget(labels_[i], i, 0);
    layout_->addWidget(input_fields_[i], i, 1);
  }
  // generate "apply" button
  QPushButton* apply_button_ = new QPushButton(this);
  apply_button_->setText("Apply");
  // connect event handler
  apply_button_->connect(apply_button_, SIGNAL(clicked()),
                         this, SLOT(on_pushButton_apply_clicked()));
  apply_button_->setFixedWidth(50);
  layout_->addWidget(apply_button_);
}

void ParamsWindow::SetActiveParams(CarParams* params) {
  if (!params) return;
  current_params_ = params;
  // grab current values
  for (size_t i = 0; i < params->description.size(); ++i) {
     input_fields_[i]->setText(QString::number(*params->value_ptrs[i]));
  }
}

void ParamsWindow::on_pushButton_apply_clicked() {
  if (!current_params_) return;
  // assign textfield values to selected parameter object
  for (size_t i = 0; i < current_params_->description.size(); ++i) {
    *current_params_->value_ptrs[i] = (input_fields_[i]->text()).toFloat();
  }
}
