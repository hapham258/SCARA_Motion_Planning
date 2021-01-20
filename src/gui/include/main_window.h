#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QVector3D>

#include <eigen3/Eigen/Dense>

#include "q_node.h"

using namespace std;
using namespace Eigen;

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private:
  Ui::MainWindow* ui;
  QNode q_node;

  QVector<double> position;
  QVector<double> velocity;
  QVector<double> acceleration;
  QVector<double> timeline;
  QVector<double> theta1;
  QVector<double> theta2;
  QVector<double> d3;
  QVector<Vector3d> path;
  double pathLength;
  double samplingPeriod;
  double duration;
  double maxVelocity;
  double radius;
  Matrix3d rotMatrix;
  Vector3d initPoint, endPoint, midPoint, centerPoint;

  const double a1 = 0.320;
  const double d1 = 0.045;
  const double a2 = 0.250;
  const double d2 = 0.125;

  void generateLSPB(double tf, double qi, double qf, double vm, int N = 1000);
  void setLine(double s, Vector3d& point);
  void setCircle(double s, Vector3d& point);
  void setJoints(const Vector3d& point, double& theta1, double& theta2, double& d3);

  void msleep(int msec);

private slots:
  void on_shoulderJointSlider_valueChanged(int value);
  void on_upperarmJointSlider_valueChanged(int value);
  void on_forearmJointSlider_valueChanged(int value);
  void on_planningButton_clicked();
  void on_runButton_clicked();

};

#endif // MAIN_WINDOW_H
