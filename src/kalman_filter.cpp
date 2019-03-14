#include <simple_kf/helper.h>
#include <simple_kf/kalman_filter.h>



namespace simple_kf
{
KalmanFilter::KalmanFilter(cv::Point3d latest_detected_position, ros::Time last_observation_time):
  latest_detected_position_(latest_detected_position),
  last_observation_time_(last_observation_time)
{
  initKalman();
  updateMeasurement(latest_detected_position, last_observation_time);
  resetState();
}

KalmanFilter::KalmanFilter(cv::Point3d latest_detected_position, ros::Time last_observation_time, int state_size, int measurement_size):
  latest_detected_position_(latest_detected_position),
  last_observation_time_(last_observation_time),
  state_size_(state_size),
  measurement_size_(measurement_size)
{
  initKalman(state_size, measurement_size);
  updateMeasurement(latest_detected_position, last_observation_time);
  resetState();
}

KalmanFilter::KalmanFilter()
{
}

void KalmanFilter::resetState()
{
  state_.at<float>(0) = measurement_.at<float>(0);
  state_.at<float>(1) = measurement_.at<float>(1);
  state_.at<float>(2) = measurement_.at<float>(2);
  state_.at<float>(3) = 0.0f;
  state_.at<float>(4) = 0.0f;
  state_.at<float>(5) = 0.0f;
 
  if(state_size_ == 9)
  {
  state_.at<float>(6) = 0.0f;
  state_.at<float>(7) = 0.0f;
  state_.at<float>(8) = 0.0f;
  }

  kf_.statePost = state_;
}

void KalmanFilter::updateMeasurement(cv::Point3d position, ros::Time stamp)
{
  // fill measurement vector with detected position values
  measurement_.at<float>(0) = position.x;
  measurement_.at<float>(1) = position.y;
  measurement_.at<float>(2) = position.z;

  latest_detected_position_ = position;
  last_observation_time_    = stamp;
}

void KalmanFilter::timeUpdateStep(double dt)
{
  // set A-Matrix here:
  if(state_size_ == 6)
  {
    kf_.transitionMatrix.at<float>(3)  = dt;
    kf_.transitionMatrix.at<float>(10) = dt;
    kf_.transitionMatrix.at<float>(17) = dt;

    // update process noise based on current ball velocity
    kf_.processNoiseCov.at<float>(0)  = std::max(10e-5, 0.1 * velocity_.x * dt);
    kf_.processNoiseCov.at<float>(7)  = std::max(10e-5, 0.1 * velocity_.y * dt);
    kf_.processNoiseCov.at<float>(14) = std::max(10e-5, 0.1 * velocity_.z * dt);
  }

  if(state_size_ == 9)
  {
    // [ 1  0  0  dt   0   0  1/2*dt^2 0        0       ]
    // [ 0  1  0   0  dt   0  0        1/2*dt^2 0       ]
    // [ 0  0  1   0   0  dt  0        0        1/2*dt^2]
    // [ 0  0  0   1   0   0  dt       0        0       ]
    // [ 0  0  0   0   1   0  0        dt       0       ]
    // [ 0  0  0   0   0   1  0        0        dt      ]
    // [ 0  0  0   0   0   0  1        0        0       ]
    // [ 0  0  0   0   0   0  0        1        0       ]
    // [ 0  0  0   0   0   0  0        0        1       ]
    double tmp = 0.5*dt*dt;
    kf_.transitionMatrix.at<float>(3)  = dt;
    kf_.transitionMatrix.at<float>(6) =  tmp;
    kf_.transitionMatrix.at<float>(13) = dt;
    kf_.transitionMatrix.at<float>(16) = tmp;
    kf_.transitionMatrix.at<float>(23) = dt;
    kf_.transitionMatrix.at<float>(26) = tmp;

    kf_.transitionMatrix.at<float>(33) = dt;
    kf_.transitionMatrix.at<float>(43) = dt;
    kf_.transitionMatrix.at<float>(53) = dt;

    //update process noise based on current ball velocity
    //TODO is this update good so? include accelerations here?
    kf_.processNoiseCov.at<float>(0)  = std::max(10e-5, 0.1 * velocity_.x * dt);
    kf_.processNoiseCov.at<float>(10) = std::max(10e-5, 0.1 * velocity_.y * dt);
    kf_.processNoiseCov.at<float>(20) = std::max(10e-5, 0.1 * velocity_.z * dt);
  }

  state_ = kf_.predict();

  time_stamp_ = ros::Time::now();
  velocity_.x = getPredictedVelocity().x;
  velocity_.y = getPredictedVelocity().y;
  velocity_.z = getPredictedVelocity().z;

  position_.x = getPredictedPosition().x;
  position_.y = getPredictedPosition().y;
  position_.z = getPredictedPosition().z;
}

void KalmanFilter::correctionStep()
{
  state_ = kf_.correct(measurement_);

  time_stamp_ = ros::Time::now();
  velocity_.x = getPredictedVelocity().x;
  velocity_.y = getPredictedVelocity().y;
  velocity_.z = getPredictedVelocity().z;

  position_.x = getPredictedPosition().x;
  position_.y = getPredictedPosition().y;
  position_.z = getPredictedPosition().z;

  if(state_size_ == 9)
  {
   acceleration_.x = getPredictedAcceleration().x;
   acceleration_.y = getPredictedAcceleration().y;
   acceleration_.z = getPredictedAcceleration().z;
  }

}

void KalmanFilter::initKalman(int state_size, int measurement_size)
{
 // if state_size == 6 -> choose constant velocity model    -->  state_ = [x, y, z, v_x, v_y, v_z]
 // if state_size == 9 -> chose non constant velocity model -->  state_ = [x, y, z, v_x, v_y, v_z, a_x, a_y, a_z]
  state_size_ = state_size;
  unsigned int type = CV_32F;

  kf_ = cv::KalmanFilter(state_size, measurement_size, 0, type);

  state_ = cv::Mat(state_size, 1, type); 

  measurement_ = cv::Mat(measurement_size, 1, type); // [z_x, z_y, z_z]
  
  if(state_size == 6)
  {
  std::cout<<"Kalman Filter model 6 is used"<<std::endl;
  // Transition State Matrix A
  // Note: set dt at each processing step!
  // [ 1  0  0  dt   0   0]
  // [ 0  1  0   0  dt   0]
  // [ 0  0  1   0   0  dt]
  // [ 0  0  0   1   0   0]
  // [ 0  0  0   0   1   0]
  // [ 0  0  0   0   0   1]
  cv::setIdentity(kf_.transitionMatrix);

  // Measurement Matrix H
  // [ 1 0 0 0 0 0]
  // [ 0 1 0 0 0 0]
  // [ 0 0 1 0 0 0]
  kf_.measurementMatrix = cv::Mat::zeros(measurement_size, state_size, type);
  kf_.measurementMatrix.at<float>(0)  = 1;
  kf_.measurementMatrix.at<float>(7)  = 1;
  kf_.measurementMatrix.at<float>(14) = 1;

  // Process Noise Covariance Matrix Q
  // [ Ex   0    0    0     0     0   ]
  // [ 0    Ey   0    0     0     0   ]
  // [ 0    0    Ez   0     0     0   ]
  // [ 0    0    0    Ev_y  0     0   ]
  // [ 0    0    0    0     Ev_y  0   ]
  // [ 0    0    0    0     0     Ev_y]
  kf_.processNoiseCov.at<float>(0)  = 10e-5;
  kf_.processNoiseCov.at<float>(7)  = 10e-5;
  kf_.processNoiseCov.at<float>(14) = 10e-5;
  kf_.processNoiseCov.at<float>(21) = 10e-4;
  kf_.processNoiseCov.at<float>(28) = 10e-4;
  kf_.processNoiseCov.at<float>(35) = 10e-4;

  // Measures Noise Covariance Matrix R
  // [0.001, 0 , 0]
  // [0, 0.001 , 0]
  // [0, 0 , 0.001]
  cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(0.001));

  // Initialize diagonal error covariance of time update with low confidence
  kf_.errorCovPre.at<float>(0)  = 0.02;
  kf_.errorCovPre.at<float>(7)  = 0.02;
  kf_.errorCovPre.at<float>(14) = 0.02;
  kf_.errorCovPre.at<float>(21) = 0.10;
  kf_.errorCovPre.at<float>(28) = 0.10;
  kf_.errorCovPre.at<float>(35) = 0.10;

  // Initialize diagonal error covariance of measurement update with low confidence
  kf_.errorCovPost.at<float>(0)  = 0.02;
  kf_.errorCovPost.at<float>(7)  = 0.02;
  kf_.errorCovPost.at<float>(14) = 0.02;
  kf_.errorCovPost.at<float>(21) = 0.10;
  kf_.errorCovPost.at<float>(28) = 0.10;
  kf_.errorCovPost.at<float>(35) = 0.10;
  }


  if(state_size == 9)   // state_size == 9 -> non constant velocity model:
  {
  std::cout<<"Kalman Filter model 9 is used"<<std::endl;
  // Transition State Matrix A
  // Note: set dt at each processing step!
  // [ 1  0  0  dt   0   0  1/2*dt^2 0        0       ]
  // [ 0  1  0   0  dt   0  0        1/2*dt^2 0       ]
  // [ 0  0  1   0   0  dt  0        0        1/2*dt^2]
  // [ 0  0  0   1   0   0  dt       0        0       ]
  // [ 0  0  0   0   1   0  0        dt       0       ]
  // [ 0  0  0   0   0   1  0        0        dt      ]
  // [ 0  0  0   0   0   0  1        0        0       ]
  // [ 0  0  0   0   0   0  0        1        0       ]
  // [ 0  0  0   0   0   0  0        0        1       ]
  cv::setIdentity(kf_.transitionMatrix);

  // Measurement Matrix H
  // [ 1 0 0 0 0 0 0 0 0]
  // [ 0 1 0 0 0 0 0 0 0]
  // [ 0 0 1 0 0 0 0 0 0]
  kf_.measurementMatrix = cv::Mat::zeros(measurement_size, state_size, type);
  kf_.measurementMatrix.at<float>(0)  = 1;
  kf_.measurementMatrix.at<float>(10) = 1;
  kf_.measurementMatrix.at<float>(20) = 1;

  // Process Noise Covariance Matrix Q
  // Wie sehr man dem System vertraut! Durch probieren herausfinden
  // small values for q -> you trust your system more
  // Messrauschen:= Modeling errors, Discretization, ...
  // If you are very confident about your equations: set Q to zero.
  // [ Ex   0    0    0     0     0     0    0    0   ]
  // [ 0    Ey   0    0     0     0     0    0    0   ]
  // [ 0    0    Ez   0     0     0     0    0    0   ]
  // [ 0    0    0    Ev_y  0     0     0    0    0   ]
  // [ 0    0    0    0     Ev_y  0     0    0    0   ]
  // [ 0    0    0    0     0     Ev_z  0    0    0   ]
  // [ 0    0    0    0     0     0     Ea_x 0    0   ]
  // [ 0    0    0    0     0     0     0    Ea_y 0   ]
  // [ 0    0    0    0     0     0     0    0    Ea_z]
  kf_.processNoiseCov.at<float>(0)  = 10e-5;
  kf_.processNoiseCov.at<float>(10) = 10e-5;
  kf_.processNoiseCov.at<float>(20) = 10e-5;
  kf_.processNoiseCov.at<float>(30) = 10e-4;   // approximation error cause of integral
  kf_.processNoiseCov.at<float>(40) = 10e-4;
  kf_.processNoiseCov.at<float>(50) = 10e-4;
  kf_.processNoiseCov.at<float>(60) = 10e-3;   // approximation error cause of integral
  kf_.processNoiseCov.at<float>(70) = 10e-3;
  kf_.processNoiseCov.at<float>(80) = 10e-3;

  // Measures Noise Covariance Matrix R
  // Wie sehr man den Messungen vertraut!
  // Miss das System in Ruhelage und berechne Standartabweichung o_xx oder 0_xx^2 (Varianz)
  // See kalman_filter folder
  // example:0.001=o_xx^2 --> o_xx = 0.031 in m --> Within +- 3.1cm are 68% of all measurements
  // [0.0008, 0 , 0]
  // [0, 0.0008 , 0]
  // [0, 0 , 0.001]
  cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(0.0008));
  kf_.measurementNoiseCov.at<float>(8) = 0.001;

  // Covariance Matrix P (Initialization error)
  // If the initialization is very good (high confidence that your states are close to the correct values) you can assume a smaller P_0 value.
  // Initialize diagonal error covariance of time update with low confidence P
  kf_.errorCovPre.at<float>(0)  = 0.02;
  kf_.errorCovPre.at<float>(10) = 0.02;
  kf_.errorCovPre.at<float>(20) = 0.02;
  kf_.errorCovPre.at<float>(30) = 0.10;
  kf_.errorCovPre.at<float>(40) = 0.10;
  kf_.errorCovPre.at<float>(50) = 0.10;
  kf_.errorCovPre.at<float>(60) = 0.50;
  kf_.errorCovPre.at<float>(70) = 0.50;
  kf_.errorCovPre.at<float>(80) = 0.50;

  // Initialize diagonal error covariance of measurement update with low confidence
  kf_.errorCovPost.at<float>(0)  = 0.02;
  kf_.errorCovPost.at<float>(10) = 0.02;
  kf_.errorCovPost.at<float>(20) = 0.02;
  kf_.errorCovPost.at<float>(30) = 0.10;
  kf_.errorCovPost.at<float>(40) = 0.10;
  kf_.errorCovPost.at<float>(50) = 0.10;
  kf_.errorCovPost.at<float>(60) = 0.50;
  kf_.errorCovPost.at<float>(70) = 0.50;
  kf_.errorCovPost.at<float>(80) = 0.50;
  }
} // end of init Kalman


void KalmanFilter::setCovariance(double x, double y, double z, double angle_x, double angle_y, double angle_z)
{
  // TODO
  covariance_[0] = x;
  covariance_[1] = y;
  covariance_[2] = z;
  covariance_[3] = angle_x; // roll pitch yaw cov values
  covariance_[4] = angle_y;
  covariance_[5] = angle_z;
}

void KalmanFilter::getCovariance(double& x, double& y, double& z, double& angle_x, double& angle_y, double& angle_z) const
{
  x = covariance_[0];
  y = covariance_[1];
  z = covariance_[2];
  angle_x = covariance_[3];
  angle_y = covariance_[4];
  angle_z = covariance_[5];
}

} // end of namespace
