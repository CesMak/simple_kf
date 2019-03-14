#include <simple_kf/vis_node.h>

#include <tf/tf.h>

#include <simple_kf/helper.h>



namespace simple_kf
{
VisNode::VisNode(ros::NodeHandle& nh)
{
  vis_pub_ = nh.advertise<visualization_msgs::Marker>( "simple_kf_vis/markers", 0 ); // for indigo!
  vis_sub_ = nh.subscribe("/cam_pose_in_world_kf", 1, &VisNode::kf_input_Cb, this);
}

VisNode::~VisNode()
{
}

void VisNode::kf_input_Cb(alfons_msgs::KfState msg)
{
  // generate new markers
  visualization_msgs::Marker sphere_marker;
  visualization_msgs::Marker cov_markersxy;
  visualization_msgs::Marker cov_markersyz;
  visualization_msgs::Marker text_marker;

    // create ball vis
    sphere_marker.action = visualization_msgs::Marker::ADD;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.ns = "simple_kf_world";
    sphere_marker.id = 1;
    sphere_marker.header = msg.header;
    sphere_marker.pose.position = msg.position_kf;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
    
    sphere_marker.color.r = 0;
    sphere_marker.color.g = 1;
    sphere_marker.color.b = 0;
    sphere_marker.color.a = 1;

    // draw covariance XY as disc
   //covariance: [0.0005032577319070697, 0.0005032577319070697, 0.0005032577319070697, 0.0, -0.0, 0.0]
   //confidence: 1.0
    cov_markersxy = sphere_marker;
    cov_markersxy.ns = "covariancexy";
    float dataxy[4] = {std::abs(static_cast<float>(msg.covariance_position[0])), 0.0f, 0.0f, std::abs(static_cast<float>(msg.covariance_position[1]))};
    cv::Mat cov2dxy = cv::Mat(2, 2, CV_32F, dataxy);
    //std::cout<<"cov2d_xy"<<cov2dxy<<std::endl;
    cv::RotatedRect rectxy = get2DErrorEllipse(cv::Point2f(msg.position_kf.x, msg.position_kf.y), cov2dxy, 5.991);
    // @TODO why turn orientation -
    //cov_markersxy.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(state.covariance[3], state.covariance[4], state.covariance[5] - rectxy.angle);
    cov_markersxy.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    cov_markersxy.scale.x = rectxy.size.height;
    cov_markersxy.scale.y = rectxy.size.width;
    cov_markersxy.scale.z = 0.01;
    //cov_markersxy.color = visual_tools_->getColor(rviz_visual_tools::RED);
    cov_markersxy.color.r = 1;
    cov_markersxy.color.g = 0;
    cov_markersxy.color.b = 0;
    cov_markersxy.color.a = 0.5;


   // draw covariance YZ as disc
    cov_markersyz = sphere_marker;
    cov_markersyz.ns = "covarianceyz";
    float data[4] = {std::abs(static_cast<float>(msg.covariance_position[1])), 0.0f, 0.0f, std::abs(static_cast<float>(msg.covariance_position[2]))};
    cv::Mat cov2d = cv::Mat(2, 2, CV_32F, data);
    //std::cout<<"cov2d_yz"<<cov2d<<std::endl;
    cv::RotatedRect rect = get2DErrorEllipse(cv::Point2f(msg.position_kf.y, msg.position_kf.z), cov2d, 5.991);
    // orientation turn that lies in yz plane!
    cov_markersyz.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57079632679, 0.0, 1.57079632679); // 1.57079632679
    cov_markersyz.scale.x = rect.size.width;  // let it like this!
    cov_markersyz.scale.y = rect.size.height;
    cov_markersyz.scale.z = 0.01;
    //cov_markersyz.color = visual_tools_->getColor(rviz_visual_tools::BLUE);
    cov_markersyz.color.r = 0;
    cov_markersyz.color.g = 0;
    cov_markersyz.color.b = 1;
    cov_markersyz.color.a = 0.5;

    // add text
    text_marker = sphere_marker;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns = "label";
    text_marker.pose.position.x += 0.05 * 1;
    text_marker.pose.position.y -= 0.05 * 1;
    text_marker.pose.position.z += 0.05 * 1;
    text_marker.scale.x = 0.025;
    text_marker.scale.y = 0.025;
    text_marker.scale.z = 0.025;
    // text_marker.color = visual_tools_->getColor(rviz_visual_tools::WHITE);
     text_marker.color.r = 1;
     text_marker.color.g = 1;
     text_marker.color.b = 1;
    text_marker.text = "kf_p";
  

  vis_pub_.publish(sphere_marker);
  vis_pub_.publish(cov_markersxy);
  vis_pub_.publish(cov_markersyz);
  vis_pub_.publish(text_marker);

} // end of function
} // end of namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_kf_vis_node");

  ros::NodeHandle nh;

  simple_kf::VisNode node(nh);
  ros::spin();

  return 0;
}