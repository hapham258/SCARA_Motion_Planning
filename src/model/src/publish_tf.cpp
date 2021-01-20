#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <vector>
#include <iostream>

const double a1 = 0.320;
const double d1 = 0.045;
const double a2 = 0.250;
const double d2 = 0.125;

void JointStatesCallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
  using namespace tf;

  auto timestamp = msg->header.stamp;
  double theta1 = msg->position[0];
  double theta2 = msg->position[1];
  double d3 = msg->position[2];

  Matrix3x3 rotation01(cos(theta1), -sin(theta1),  0,
                       sin(theta1),  cos(theta1),  0,
                                 0,            0,  1);
  Vector3 translation01(a1*cos(theta1),
                        a1*sin(theta1),
                                   d1);
  Matrix3x3 rotation12(cos(theta2), -sin(theta2),  0,
                       sin(theta2),  cos(theta2),  0,
                                 0,            0,  1);
  Vector3 translation12(a2*cos(theta2),
                        a2*sin(theta2),
                                   d2);
  Matrix3x3 rotation23(1,  0,  0,
                       0, -1,  0,
                       0,  0, -1);
  Vector3 translation23(  0,
                          0,
                        -d3);

  std::vector<StampedTransform> transforms;
  StampedTransform footprint_to_base(Transform(Quaternion(0,0,0,1), Vector3(0, 0, 0.20)), timestamp, "footprint", "base");
  StampedTransform transform01(Transform(rotation01, translation01), timestamp, "base", "shoulder");
  StampedTransform transform12(Transform(rotation12, translation12), timestamp, "shoulder", "upperarm");
  StampedTransform transform23(Transform(rotation23, translation23), timestamp, "upperarm", "forearm");
  StampedTransform forearm_to_tool(Transform(Quaternion(0,0,0,1), Vector3(0, 0, 0.25)), timestamp, "forearm", "tool");
  transforms.push_back(footprint_to_base);
  transforms.push_back(transform01);
  transforms.push_back(transform12);
  transforms.push_back(transform23);
  transforms.push_back(forearm_to_tool);

  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(transforms);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("joint_states", 1, &JointStatesCallBack);

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
