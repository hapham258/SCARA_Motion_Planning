#ifndef Q_NODE_H
#define Q_NODE_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include <QThread>

using namespace sensor_msgs;
using namespace visualization_msgs;

class QNode : public QThread
{
  Q_OBJECT

public:
  QNode();
  ~QNode();

  ros::Publisher pubJoints;
  ros::Publisher pubPathMarker;

  int pathId = 0;
  JointState joints;

  void init();
  void run();

  void visualizePath(const double* srcPoint, const double* destPoint);

signals:
  void rosShutdown();
};

#endif // Q_NODE_H
