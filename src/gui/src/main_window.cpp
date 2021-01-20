#include "main_window.h"
#include "ui_main_window.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  ui->positionPlot->addGraph();
  ui->positionPlot->graph(0)->setPen(QPen(Qt::blue));
  ui->positionPlot->graph(0)->setName("Set Value");
  ui->positionPlot->yAxis->setLabel("Position (m)");

  ui->velocityPlot->addGraph();
  ui->velocityPlot->graph(0)->setPen(QPen(Qt::blue));
  ui->velocityPlot->graph(0)->setName("Set Value");
  ui->velocityPlot->yAxis->setLabel("Velocity (m/s)");

  ui->accelerationPlot->addGraph();
  ui->accelerationPlot->graph(0)->setPen(QPen(Qt::blue));
  ui->accelerationPlot->graph(0)->setName("Set Value");
  ui->accelerationPlot->xAxis->setLabel("Time (s)");
  ui->accelerationPlot->yAxis->setLabel("Acceleration (m/s^2)");

  connect(&q_node, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::msleep(int msec)
{
  QEventLoop loop;
  QTimer::singleShot(msec, &loop, &QEventLoop::quit);
  loop.exec();
}

void MainWindow::generateLSPB(double tf, double qi, double qf, double vm, int N)
{
  double tc = (qi - qf + vm * tf) / vm;
  double am = vm * vm / (qi - qf + vm * tf);
  samplingPeriod = tf / (N - 1);
  int nc = round(tc / samplingPeriod);

  position.clear();
  velocity.clear();
  acceleration.clear();
  timeline.clear();
  for (int n = 0; n < N; n++)
  {
    double t = n * samplingPeriod;
    timeline.push_back(t);
    if (n < nc)
    {
      position.push_back(qi + 0.5 * am * t * t);
      velocity.push_back(am * t);
      acceleration.push_back(am);
    }
    else if (n < N - 1 - nc)
    {
      position.push_back(qi + am * tc * (t - tc / 2));
      velocity.push_back(am * tc);
      acceleration.push_back(0);
    }
    else
    {
      position.push_back(qf - 0.5 * am * (tf - t) * (tf - t));
      velocity.push_back(am * (tf - t));
      acceleration.push_back(-am);
    }
  }
}

void MainWindow::setLine(double s, Vector3d& point) { point = initPoint + s / pathLength * (endPoint - initPoint); }

void MainWindow::setCircle(double s, Vector3d& point)
{
  Vector3d point_(radius * cos(s / radius), radius * sin(s / radius), 0);
  point = centerPoint + rotMatrix * point_;
}

void MainWindow::setJoints(const Vector3d& point, double& theta1, double& theta2, double& d3)
{
  double px = static_cast<double>(point.x());
  double py = static_cast<double>(point.y());
  double pz = static_cast<double>(point.z());

  d3 = d1 + d2 - (pz - 0.22 / 2 - 0.09 + 0.25);

  double c2 = (pow(px, 2) + pow(py, 2) - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2);
  double s2 = sqrt(1 - pow(c2, 2));
  theta2 = atan2(s2, c2);

  double c1 = (px * (a1 + a2 * c2) + py * a2 * s2) / (pow(a1, 2) + pow(a2, 2));
  double s1 = (py * (a1 + a2 * c2) - px * a2 * s2) / (pow(a1, 2) + pow(a2, 2));
  theta1 = atan2(s1, c1);
}

void MainWindow::on_shoulderJointSlider_valueChanged(int value) { q_node.joints.position[0] = value * M_PI / 180; }

void MainWindow::on_upperarmJointSlider_valueChanged(int value) { q_node.joints.position[1] = value * M_PI / 180; }

void MainWindow::on_forearmJointSlider_valueChanged(int value) { q_node.joints.position[2] = value / 100.0; }

void MainWindow::on_planningButton_clicked()
{
  initPoint = Vector3d(ui->initXlineEdit->text().toDouble(), ui->initYlineEdit->text().toDouble(),
                       ui->initZlineEdit->text().toDouble());
  midPoint = Vector3d(ui->midXlineEdit->text().toDouble(), ui->midYlineEdit->text().toDouble(),
                      ui->midZlineEdit->text().toDouble());
  endPoint = Vector3d(ui->endXlineEdit->text().toDouble(), ui->endYlineEdit->text().toDouble(),
                      ui->endZlineEdit->text().toDouble());
  duration = ui->durationlineEdit->text().toDouble();
  maxVelocity = ui->maxVelocitylineEdit->text().toDouble();

  if (ui->lineBox->isChecked())
  {
    Vector3d lengthVector = endPoint - initPoint;
    pathLength = lengthVector.norm();
  }

  if (ui->circleBox->isChecked())
  {
    Vector3d p1 = initPoint;
    Vector3d p2 = midPoint;
    Vector3d p3 = endPoint;

    Vector3d w = (p1 - p2).cross(p1 - p3);
    Matrix3d A;
    A << w, 2 * (p1 - p2), 2 * (p1 - p3);
    Vector3d b(w.dot(p1), p1.dot(p1) - p2.dot(p2), p1.dot(p1) - p3.dot(p3));
    Vector3d pc = A.transpose().inverse() * b;
    centerPoint = pc;

    Vector3d Rz = w;
    Vector3d Rx = p1 - pc;
    Vector3d Ry = Rz.cross(Rx);
    Matrix3d R;
    R << Rx / Rx.norm(), Ry / Ry.norm(), Rz / Rz.norm();
    rotMatrix = R;

    radius = (p1 - pc).norm();
    Vector3d p1_ = R.inverse() * (p1 - pc);
    Vector3d p3_ = R.inverse() * (p3 - pc);
    double theta = acos(p1_.dot(p3_) / (radius * radius));
    theta = (p3_(1) < 0) ? 2 * M_PI - theta : theta;
    pathLength = theta * radius;
  }

  ui->notificationEdit->append("Path length is " + QString().setNum(pathLength) + " m.");
  double lowerBoundVelocity = pathLength / duration;
  double upperBoundVelocity = 2 * pathLength / duration;
  ui->notificationEdit->append("Maximum velocity must be between " + QString().setNum(lowerBoundVelocity) +
                               " m/s and " + QString().setNum(2 * upperBoundVelocity) + " m/s.");

  maxVelocity = isgreater(maxVelocity, upperBoundVelocity) ? upperBoundVelocity : maxVelocity;
  maxVelocity = isless(maxVelocity, lowerBoundVelocity) ? lowerBoundVelocity : maxVelocity;
  generateLSPB(duration, 0, pathLength, maxVelocity);

  ui->positionPlot->graph(0)->setData(timeline, position);
  ui->velocityPlot->graph(0)->setData(timeline, velocity);
  ui->accelerationPlot->graph(0)->setData(timeline, acceleration);
  ui->positionPlot->rescaleAxes();
  ui->positionPlot->replot();
  ui->velocityPlot->rescaleAxes();
  ui->velocityPlot->replot();
  ui->accelerationPlot->rescaleAxes();
  ui->accelerationPlot->replot();
}

void MainWindow::on_runButton_clicked()
{
  theta1.clear();
  theta2.clear();
  d3.clear();
  path.clear();

  for (int i = 0; i < position.size(); i++)
  {
    path.push_back(Vector3d());
    if (ui->lineBox->isChecked())
      setLine(position[i], path.back());
    if (ui->circleBox->isChecked())
      setCircle(position[i], path.back());

    double _theta1, _theta2, _d3;
    setJoints(path.back(), _theta1, _theta2, _d3);
    theta1.push_back(_theta1);
    theta2.push_back(_theta2);
    d3.push_back(_d3);
  }

  q_node.pathId = 0;
  for (int i = 0; i < position.size(); i++)
  {
    q_node.joints.position[0] = theta1[i];
    q_node.joints.position[1] = theta2[i];
    q_node.joints.position[2] = d3[i];
    if (i)
      q_node.visualizePath(path[i - 1].data(), path[i].data());

    msleep(samplingPeriod * 1000);
  }
}
