#ifndef VIS_NODE_H__
#define VIS_NODE_H__

#include <ros/ros.h>

//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <alfons_msgs/KfState.h>



namespace simple_kf
{
class VisNode
{
public:
  ros::Publisher vis_pub_; // for indigo!

  VisNode(ros::NodeHandle& nh);
  virtual ~VisNode();

protected:
  void kf_input_Cb(alfons_msgs::KfState msg);
  // just works under kinetic do not use it with indigo.
  //rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // subscriber
  ros::Subscriber vis_sub_;
};
}

#endif
