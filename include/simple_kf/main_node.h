#ifndef SIMPLE_KF_NODE_H__
#define SIMPLE_KF_NODE_H__

// messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>


namespace simple_kf
{
class SimpleKfNode
{
public:
  SimpleKfNode(ros::NodeHandle& nh);
  virtual ~SimpleKfNode();

  void update();

protected:
  // ROS API callbacks
  void inputPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  void predictionStep(double dt);

  // class members
  geometry_msgs::PoseWithCovarianceStampedConstPtr input_pose_;
  double dt_;
  int kf_states_;
  KalmanFilter* kf_;

  // subscriber

  // publisher

  // action server

  // consts:  (constexpr required for double since cpp11)

}; // end of class
}

#endif
