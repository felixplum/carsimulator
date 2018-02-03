#ifndef PARAMS_WINDOW_H
#define PARAMS_WINDOW_H

#include <QWidget>
#include <QGridLayout>
#include <QLineEdit>
#include <QDoubleValidator>
#include <QPushButton>
#include <QLabel>
#include <car.h>

class ParamsWindow : public QWidget
{
  Q_OBJECT
public:
  explicit ParamsWindow(QWidget *parent = nullptr);
  void SetActiveParams(CarParams* params);
signals:

public slots:
  void on_pushButton_apply_clicked();
private:
  QGridLayout *layout_;
  std::vector<QLineEdit*> input_fields_;
  std::vector<QLabel*> labels_;
  std::vector<float*> param_valueptrs_;
  CarParams* current_params_;
  QPushButton* apply_button_;

};

#endif // PARAMS_WINDOW_H
