#include "q_node.h"

QNode::QNode() { init(); }

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

void QNode::init()
{
  int argc;
  char** argv = nullptr;
  ros::init(argc, argv, "qnode");
  ros::start();

  ros::NodeHandle nh;
  pubJoints = nh.advertise<JointState>("joint_states", 1);
//  pubPathMarker = nh.advertise<Marker>("path_marker", 1, true);

  joints.position.resize(3, 0);

  start();
}

void QNode::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    joints.header.stamp = ros::Time::now();
    pubJoints.publish(joints);

    ros::spinOnce();
    loop_rate.sleep();
  }
  emit rosShutdown();
}

void QNode::visualizePath(const double* srcPoint, const double* destPoint)
{
  Marker marker;
  marker.header.frame_id = "footprint";
  marker.header.stamp = ros::Time::now();
  marker.ns = "path";
  marker.id = pathId++;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(2);
  marker.points[0].x = srcPoint[0];
  marker.points[0].y = srcPoint[1];
  marker.points[0].z = srcPoint[2];
  marker.points[1].x = destPoint[0];
  marker.points[1].y = destPoint[1];
  marker.points[1].z = destPoint[2];
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.005;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

//  pubPathMarker.publish(marker);
}
