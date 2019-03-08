#include <ball_tracking_rgb/detector.h>
#include <ball_tracking_rgb/helper.h>



namespace ball_tracking_rgb
{
Detector::Detector(const sensor_msgs::CameraInfo& camera_info, double ball_diameter, double max_age,
                   double min_radius, double fill_ratio, bool use_simulation, int kf_states)
  : ball_uid_counter_(0)
  , ball_diameter_(ball_diameter)
  , max_age_(max_age)
  , min_radius_(min_radius)
  , fill_ratio_(fill_ratio)
  , use_simulation_(use_simulation)
  , kf_states_(kf_states)
{
  pinhole_camera_.fromCameraInfo(camera_info);
}

void Detector::update(const cv::Mat& img, const ros::Time& stamp, double dt)
{
  // detect balls in image
  std::vector<BallPercept> percepts = detectBalls(img, stamp);

  // Kalman prediction step
  predictionStep(dt);

  // Kalman correction step
  correctionStep(percepts, stamp);

  ball_models_next_val = getBallModels();
  // get future value for BoP (3 time steps a 60 Hz -> predics  0.050000000000000 s in the future!
  // calcNextPosition(1*dt);
}

std::vector<Detector::BallPercept> Detector::detectBalls(const cv::Mat& img, const ros::Time& stamp)
{
  ball_models_raw.clear();
  cv::Mat temp;

  // noise smoothing
  cv::GaussianBlur(img, temp, cv::Size(5, 5), 3.0, 3.0);

  // HSV conversion
  cv::cvtColor(temp, temp, CV_BGR2HSV);

  // color thresholding
  binary_img_ = cv::Mat::zeros(temp.size(), CV_8UC1);
  cv::inRange(temp, cv::Scalar(min_hue_, min_sat_, min_val_),
                    cv::Scalar(max_hue_, max_sat_, max_val_),
              binary_img_);

  // improving the result
  cv::erode(binary_img_, binary_img_, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(binary_img_, binary_img_, cv::Mat(), cv::Point(-1, -1), 2);

  // contours detection
  std::vector<std::vector<cv::Point>> contours;
  // find contours works on the the binary_img_work and changes it!
  cv::Mat binary_img_work_ = binary_img_.clone();
  cv::findContours(binary_img_work_, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // data structure to store all detections from image
  std::vector<BallPercept> percepts;

  // min enclosing circle
  std::vector<cv::Point2f> center(contours.size());
  std::vector<float> radius(contours.size());
  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::minEnclosingCircle(contours[i], center[i], radius[i]);

    double fill_ratio = cv::contourArea(contours[i]) / (radius[i]*radius[i]*M_PI);
    double diameter = radius[i]*2.0;

    // let the minimum radius be 10px -> max. distance ~= 2m
    if (radius[i] > min_radius_ && fill_ratio > fill_ratio_)
    {
      BallPercept p;
      p.stamp = stamp;
      p.pos = projectBallPerceptTo3d(center[i], diameter);
      p.diameter_px = diameter;
      percepts.push_back(p);

    // store raw data as well:
    BallModel::Ptr ball_model_raw(new BallModel());
    ball_model_raw->setUId(i);
    ball_model_raw->setMatched(false);
    ball_model_raw->setPosition(p.pos.x, p.pos.y, p.pos.z);
    ball_model_raw->setDiameter(ball_diameter_);
    ball_model_raw->setDiameterPx(p.diameter_px);
    ball_model_raw->setLastObservationTimestamp(p.stamp);
    ball_model_raw->setLastDetectedPosition(p.pos);
    ball_models_raw.push_back(ball_model_raw);
    }
  }

  return percepts;
}

void Detector::resetKalmanFilters()
{
  for (BallModel::Ptr ball_model : ball_models_)
  {
    KalmanFilter &kf = kfilters_[ball_model];
    kfilters_[ball_model] = KalmanFilter(ball_model, kf_states_, 3); // for model with 9 states
  }
}


void Detector::calcNextPosition(double dt)
{
  for (BallModel::Ptr ball_model : ball_models_next_val)
  {
    KalmanFilter& kf = kfilters_[ball_model];

    kf.timeUpdateStep(dt);

    //cv::Mat cov = kf.getErrorCovPost(); //6x6 or 9x9

    //ball_model->setErrorEllipseXY(get2DErrorEllipse(cv::Point2f(ball_model->getPredictedPosition().x, ball_model->getPredictedPosition().y), cov2dxy, 5.991));
    //ball_model->setErrorEllipseYZ(get2DErrorEllipse(cv::Point2f(ball_model->getPredictedPosition().y, ball_model->getPredictedPosition().z), cov2dyz, 5.991));
  }
}


void Detector::predictionStep(double dt)
{
  for (BallModel::Ptr ball_model : ball_models_)
  {
    KalmanFilter& kf = kfilters_[ball_model];

    kf.timeUpdateStep(dt);

    // update error ellipse
    // TODO sometimes the cov Matrix (often after 10min or so) contains nan values
    // If this is the case reinit the Kalman filter?!
    cv::Mat cov = kf.getErrorCovPost(); //6x6 or 9x9

    // handle nan/high nuber case:
    // sometimes it happens that cov values get really High -> Why is that?!
    // To prevent that just reinit the kalman filter.
    cv::Mat mask = cv::Mat(cov != cov); // check for nan
    if(cv::sum(cov)[0]>100.0 || cv::sum(mask)[0]>0)
    {
         std::cout<<"getErrorCovPost cov mat "<<cov<<std::endl<<" Sum: "<<cv::sum(cov)[0]<<std::endl;
        kfilters_[ball_model] = KalmanFilter(ball_model, kf_states_, 3); // for model with 9 states
    }

    // Custom Cov to test uncertaintys:
//    double pos_x=cov_kal.at<float>(0);
//    double pos_y=cov_kal.at<float>(7)*5;
//    double pos_z=cov_kal.at<float>(14)*20;
//    double time_val=dt;
//    double error_vel=0.13245623;
//       cv::Mat cov = (cv::Mat_<float>(6,6) <<
//                      pos_x, 0, 0, time_val, 0, 0,
//                      0, pos_y, 0, 0, time_val, 0,
//                      0, 0, pos_z, 0, 0, time_val,
//                      time_val, 0, 0, error_vel, 0, 0,
//                      0, time_val, 0, 0, error_vel, 0,
//                      0, 0, time_val, 0, 0, error_vel );

//    CovPost example:
//       [0.00050343538, 0, 0, 0.033616263, 0, 0;
//         0, 0.00050343538, 0, 0, 0.033616263, 0;
//         0, 0, 0.00050343538, 0, 0, 0.033616263;
//         0.033616263, 0, 0, 0.13245623, 0, 0;
//         0, 0.033616263, 0, 0, 0.13245623, 0;
//         0, 0, 0.033616263, 0, 0, 0.13245623]

    cv::Mat cov2dxy = cov(cv::Range(0, 2), cv::Range(0, 2)); // use only sub covariance of position state (2x2)
    cv::Mat cov2dyz = cov(cv::Range(1, 3), cv::Range(1, 3));
//    Cov2D Example: (just x, y position)
//    [0.00050338131, 0;
//      0, 0.00050338131]

    ball_model->setErrorEllipseXY(get2DErrorEllipse(cv::Point2f(ball_model->getPredictedPosition().x, ball_model->getPredictedPosition().y), cov2dxy, 5.991));
    ball_model->setErrorEllipseYZ(get2DErrorEllipse(cv::Point2f(ball_model->getPredictedPosition().y, ball_model->getPredictedPosition().z), cov2dyz, 5.991));

    // entfernung und verschiebung zur kamera ist x,y,z
//    if(ball_model->getPredictedPosition().z>10.0||ball_model->getPredictedPosition().x>1.0||ball_model->getPredictedPosition().y>1.0)
//    {
//        std::cout<<"cov "<<cov<<std::endl; // <- This is all nan!!! WHY?!

////        cv::Mat K = kf.getGain();
////        cv::Mat H = kf.getMeasurement();
////        std::cout<<"K:"<<K<<std::endl;
////        std::cout<<"H:"<<H<<std::endl;
//    }

    // Covariance is set in ball.cpp
    if(kf.getStateSize() == 6)
    {
        ball_model->setCovariance(cov.at<float>(0), cov.at<float>(7), cov.at<float>(14), 0.0, 0.0, ball_model->getErrorEllipseXY().angle); /// @TODO: Works only for axis aligned covariance
    }

    if(kf.getStateSize() == 9)
    {   // TODO!!!
        ball_model->setCovariance(cov.at<float>(0), cov.at<float>(10), cov.at<float>(20), 0.0, 0.0, ball_model->getErrorEllipseXY().angle); /// @TODO: Works only for axis aligned covariance
    }

    ball_model->setConfidence(1.0); /// @TODO: Compute feasible confidence
  }
}

void Detector::correctionStep(const std::vector<BallPercept>& ball_percepts, const ros::Time& stamp)
{
  std::vector<BallPercept> percepts = ball_percepts;

  // checked if matched ball
  for (BallModel::Ptr ball_model : ball_models_)
  {
    ball_model->setMatched(false);

    int idx = getClosestBallPercept(percepts, ball_model);

    if (idx != -1) // percept canditate found
    {
      // get corresponding kalman filter
      KalmanFilter& kf = kfilters_[ball_model];

      // perform correction step
      // should the nex t line go here? TODO?!
      ball_model->setDiameterPx(percepts[idx].diameter_px);
      kf.updateMeasurement(percepts[idx].pos, percepts[idx].stamp);
      kf.correctionStep();

      // mark ball as matched
      ball_model->setMatched(true);

      // remove percept from list
      percepts.erase(percepts.begin() + idx);
    }
  }

  // for each unmatched percept just create a new ball model
  for (const BallPercept& percept : percepts)
  {
    BallModel::Ptr ball_model(new BallModel());
    ball_model->setUId(ball_uid_counter_++);
    ball_model->setMatched(true);
    ball_model->setPosition(percept.pos.x, percept.pos.y, percept.pos.z);
    ball_model->setDiameter(ball_diameter_);
    ball_model->setDiameterPx(percept.diameter_px);
    ball_model->setLastObservationTimestamp(percept.stamp);
    ball_model->setLastDetectedPosition(percept.pos);
    ball_models_.push_back(ball_model);

    //kfilters_[ball_model] = KalmanFilter(ball_model); // for model with 6
    // kf_states_ is set in config / yaml file.
    kfilters_[ball_model] = KalmanFilter(ball_model, kf_states_, 3); // for model with 9 states
  }

  // removed ball models which hasn't matched a specific time
  for (std::vector<BallModel::Ptr>::iterator itr = ball_models_.begin(); itr != ball_models_.end();)
  {
    BallModel::Ptr ball = *itr;

    if (!ball->isMatched() && (stamp - ball->getLastObservationTimestamp()).toSec() > max_age_)
      itr = ball_models_.erase(itr);
    else
      itr++;
  }
}

/**
 * @brief Detector::projectBallPerceptTo3d
 * See also: http://docs.ros.org/diamondback/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html:
 * 00259 cv::Point3d PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect) const
   00260 {
   00261   assert( initialized() );
   00262
   00263   cv::Point3d ray;
   00264   ray.x = (uv_rect.x - cx() - Tx()) / fx();
   00265   ray.y = (uv_rect.y - cy() - Ty()) / fy();
   00266   ray.z = 1.0;
   00267   return ray;
   00268 }
 * @param p
 * @param diameter
 * @return
 */
cv::Point3d Detector::projectBallPerceptTo3d(const cv::Point2d& p, double diameter) const
{
  // This method should do basically the same as
  // calculating the z distance just with:
  // point.z=(225*ball_diameter_)/(diameter/2.0*100);
  // TODO but why is to the x and y position also added 0.5*(distance_x + distance_y)

  double distance_x = (ball_diameter_ * pinhole_camera_.fx()) / diameter; // [m] = in simulation: 0.969799
  double distance_y = (ball_diameter_ * pinhole_camera_.fy()) / diameter; // [m] = in simulation: 0.969799

  cv::Point3d point = pinhole_camera_.projectPixelTo3dRay(p);
  // TODO check this code is the z - distance calculated always correclty?! even without knowing the correct focal length??!
  //std::cout<<"vorher: "<<point<<std::endl; //   vorher: [-0.00328222, 0.0109407, 1]
  point = point * 0.5 * (distance_x + distance_y);
  //std::cout<<"nachher point: "<<point<<std::endl; // nachher point: [-0.00639991, 0.021333, 1.94987]

  // This conversion: x=z, y=-x, z=-y -->   return cv::Point3d(point.z, -point.x, -point.y);
  // Another conversion: x=x, y=z z = y -->   return cv::Point3d(point.x, point.z, point.y);
  // Use another conversion cause of extrinsic camera calibration has this coordinate frame!
  // is basically the conversion from the rgb_optical_frame to the rgb_frame link! btw. the camera_link
  // cause the camera_info message is in the rgb_optical_frame but we want the point relative to the camera_link coordinate system!

  // in the simulation things are different the camera coordinate system and the camera_link system are not the same!
  //return cv::Point3d(point.x, point.y, point.z);
  if(use_simulation_)
  {
    return cv::Point3d(point.z, -point.x, -point.y);
  }
  else
  {
    return cv::Point3d(point.x, point.y, point.z);
  }

}

int Detector::getClosestBallPercept(const std::vector<BallPercept>& ball_percepts, BallModel::ConstPtr ball_model)
{
  int idx = -1;
  double min_dist_sq = std::numeric_limits<double>::max();

  // determine closest percept within the current error ellipse of the given ball model
  // @TODO do I also have to adapt this method for yz plane?!
  for (int i = 0; i < ball_percepts.size(); i++)
  {
    cv::Point2f percept_position = cv::Point2f(ball_percepts[i].pos.x, ball_percepts[i].pos.y);
    cv::RotatedRect ellipse = ball_model->getErrorEllipseXY();

    if( (ellipse.size.width != ellipse.size.width)  || (ellipse.size.height != ellipse.size.height)) // check for nan value!
    {

        std::cout<<"NAN CASE in getClosestBallPercept set error ellipse to a 0.12 size"<<std::endl;
        std::cout<<ellipse.size<<std::endl;

        ellipse.size.width = 0.12; // this handling might not be correct!
        ellipse.size.height = 0.12;
    }

    cv::Size2f size(std::max(ellipse.size.width,  static_cast<float>(2.0*ball_diameter_)),
                    std::max(ellipse.size.height, static_cast<float>(2.0*ball_diameter_)));

    // check if current percept is within error ellipse
    if (pointInEllipse(percept_position, ellipse.center, size, ellipse.angle))
    {
      double dist_sq = getDistance2DSq(percept_position, ellipse.center);

      // check if current percept is closer than previous best candidate
      if (dist_sq < min_dist_sq)
      {
        min_dist_sq = dist_sq;
        idx = i;
      }
    }
  }
  return idx;
}
}

