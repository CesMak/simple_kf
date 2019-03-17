/**************************************************************************//**
   @author  Markus Lamprecht
   @date    March 2019
   @link    www.simact.de/about_me
   @Copyright (c) 2019 Markus Lamprecht. BSD
 *****************************************************************************/
#ifndef SIMPLE_KF_NODE_H__
#define SIMPLE_KF_NODE_H__

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
  void inputPoseCb(geometry_msgs::PoseWithCovarianceStamped msg);

  void predictionStep(double dt);
  void correctionStep();
  void publishResult();

  // class members
  geometry_msgs::PoseWithCovarianceStamped input_pose_;
  geometry_msgs::PoseWithCovarianceStamped last_processed_pose_;
  double dt_;
  int kf_states_;
  KalmanFilter* kf_;
  bool message_received_;

  // subscriber
  ros::Subscriber sub_pose1_;

  // publisher
  ros::Publisher pose_kf1_pub;

  // action server

  // consts:  (constexpr required for double since cpp11)

}; // end of class
}

#endif
