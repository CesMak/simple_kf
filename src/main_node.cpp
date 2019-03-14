#include <simple_kf/helper.h>
#include <simple_kf/kalman_filter.h>
#include <simple_kf/main_node.h>

// the library includes are in helper.h
namespace simple_kf
{

// constructor:  
SimpleKfNode::SimpleKfNode(ros::NodeHandle& nh):
  dt_(0.1),
  kf_states_(6),
  message_received_(false)
{
  cv::Point3f latest_detected_position;
  latest_detected_position.x = input_pose_.pose.pose.position.x;
  latest_detected_position.y = input_pose_.pose.pose.position.y;
  latest_detected_position.z = input_pose_.pose.pose.position.z;

  ros::Time last_observation_time = input_pose_.header.stamp;
  kf_ = new KalmanFilter(latest_detected_position, last_observation_time);

  sub_pose1_  = nh.subscribe("/cam_pose_in_world", 1, &SimpleKfNode::inputPoseCb, this);

  pose_kf1_pub = nh.advertise<alfons_msgs::KfState>("cam_pose_in_world_kf", 1);

  ROS_INFO_STREAM("simple_kf initialization finished:  dt="<<dt_<<"s");
}

void SimpleKfNode::inputPoseCb(geometry_msgs::PoseWithCovarianceStamped msg)
{
  message_received_ = true;
  input_pose_ = msg;
}

SimpleKfNode::~SimpleKfNode()
{
  delete kf_;
}

void SimpleKfNode::update()
{
  // // detect balls in image
  // std::vector<BallPercept> percepts = detectBalls(img, stamp);

  // Kalman prediction step
  if(message_received_)
  {
    predictionStep(dt_);

    correctionStep();
    publishResult();
  }
}



void SimpleKfNode::predictionStep(double dt)
{
    kf_->timeUpdateStep(dt);

    // update error ellipse
    // TODO sometimes the cov Matrix (often after 10min or so) contains nan values
    // If this is the case reinit the Kalman filter?!
    cv::Mat cov = kf_->getErrorCovPost(); //6x6 or 9x9

    // handle nan/high nuber case:
    // sometimes it happens that cov values get really High -> Why is that?!
    // To prevent that just reinit the kalman filter.
    cv::Mat mask = cv::Mat(cov != cov); // check for nan
    if(cv::sum(cov)[0]>100.0 || cv::sum(mask)[0]>0)
    {
         ROS_ERROR("TO DO overfilled cov");
         std::cout<<"TODO ERROR getErrorCovPost cov mat "<<cov<<std::endl<<" Sum: "<<cv::sum(cov)[0]<<std::endl;
         //kf_ = new KalmanFilter(latest_detected_position, last_observation_time, kf_states_, 3); // for model with 9 states
    }
    cv::Mat cov2dxy = cov(cv::Range(0, 2), cv::Range(0, 2)); // use only sub covariance of position state (2x2)
    cv::Mat cov2dyz = cov(cv::Range(1, 3), cv::Range(1, 3));
//    Cov2D Example: (just x, y position)
//    [0.00050338131, 0;
//      0, 0.00050338131]

    // the function get2DErrorEllipse is in helper.cpp
    kf_->setErrorEllipseXY(get2DErrorEllipse(cv::Point2f(kf_->getPredictedPosition().x, kf_->getPredictedPosition().y), cov2dxy, 5.991));
    kf_->setErrorEllipseYZ(get2DErrorEllipse(cv::Point2f(kf_->getPredictedPosition().y, kf_->getPredictedPosition().z), cov2dyz, 5.991));

    // Covariance is set in ball.cpp
    if(kf_->getStateSize() == 6)
    {
        kf_->setCovariance(cov.at<float>(0), cov.at<float>(7), cov.at<float>(14), 0.0, 0.0, kf_->getErrorEllipseXY().angle); /// @TODO: Works only for axis aligned covariance
    }

    if(kf_->getStateSize() == 9)
    {   // TODO!!!
        kf_->setCovariance(cov.at<float>(0), cov.at<float>(10), cov.at<float>(20), 0.0, 0.0, kf_->getErrorEllipseXY().angle); /// @TODO: Works only for axis aligned covariance
    }

    //kf_->setConfidence(1.0); /// @TODO: Compute feasible confidence
  }

void SimpleKfNode::correctionStep()
{
    cv::Point3f latest_detected_position;
    latest_detected_position.x = input_pose_.pose.pose.position.x;
    latest_detected_position.y = input_pose_.pose.pose.position.y;
    latest_detected_position.z = input_pose_.pose.pose.position.z;

    ros::Time last_observation_time = input_pose_.header.stamp;

    kf_->updateMeasurement(latest_detected_position, last_observation_time);
    kf_->correctionStep();
}

void SimpleKfNode::publishResult()
{
  cv::Point3f position_kf = kf_->getKfPosition();
  cv::Point3f velocity_kf = kf_->getKfVelocity();

  alfons_msgs::KfState msg;

  msg.header.stamp =  ros::Time::now();
  msg.header.frame_id = "world";
  // input position:
  msg.input_position.x = input_pose_.pose.pose.position.x;
  msg.input_position.y = input_pose_.pose.pose.position.y;
  msg.input_position.z = input_pose_.pose.pose.position.z;

  msg.position_kf.x = position_kf.x;
  msg.position_kf.y = position_kf.y;
  msg.position_kf.z = position_kf.z;

  msg.velocity_kf.x = velocity_kf.x;
  msg.velocity_kf.y = velocity_kf.y;
  msg.velocity_kf.z = velocity_kf.z;
 
  kf_->getCovariance(msg.covariance_position[0],
                     msg.covariance_position[1],  
                     msg.covariance_position[2],  
                     msg.covariance_position[3],  
                     msg.covariance_position[4],         
                     msg.covariance_position[5]);          
                                      

    if (pose_kf1_pub.getNumSubscribers() > 0)
      pose_kf1_pub.publish(msg);    
}



} //end of namespace






int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_kf");

  ros::NodeHandle nh;

  simple_kf::SimpleKfNode node(nh);
  ros::Rate loop_rate(nh.param("simple_kf/update_rate", 10.0));

  while (ros::ok())
  {
    ros::spinOnce();
    node.update();

    if (!loop_rate.sleep())
      ROS_WARN_THROTTLE(10.0, "[simple_kf] Update rate was not met!");
  }

  return 0;
}