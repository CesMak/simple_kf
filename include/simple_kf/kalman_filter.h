#ifndef KALMAN_FILTER_H__
#define KALMAN_FILTER_H__

#include <ros/ros.h>

#include <opencv2/video/video.hpp>

namespace simple_kf
{
class KalmanFilter
{
public:
  /**
   * @brief Constructor of KalmanFilter class
   * @param ball_model associated ball model which state is estimated using a Kalman filter
   */
  KalmanFilter(cv::Point3d latest_detected_position, ros::Time last_observation_time);

  /**
   * @brief Constructor of KalmanFilter class with model selection!
   * @param ball_model associated ball model which state is estimated using a Kalman filter
   * @param state_size_ number of states   options: [6,9]
   * @param measurement_size_ number of measured states options: [3]
   */
  KalmanFilter(cv::Point3d latest_detected_position, ros::Time last_observation_time, int state_size, int measurement_size);
  
  /**
   * @brief Destructor
   */
  KalmanFilter();

  /**
   * @brief Resets state to current measurement
   */
  void resetState();

  /**
   * @brief Updates the Measurement state[x, y, z, vx, vy, vz] vector with new measurement values
   * @param position position of detected position of ball [m]
   * @param stamp time stamp when detection was done
   */
  void updateMeasurement(cv::Point3d position, ros::Time stamp);

  /**
   * @brief Performs time update on Kalman filter after transition matrix
   * (and if implemented processNoiseCov) update and updates ball model
   * @param dt time since last call
   */
  void timeUpdateStep(double dt);

  /**
   * @brief Performs measurement update on Kalman filter and update ball model
   */
  void correctionStep();

  // Getter and setter
  cv::Point3f getPredictedPosition() const { return cv::Point3f(state_.at<float>(0), state_.at<float>(1), state_.at<float>(2)); }
  cv::Point3f getPredictedVelocity() const { return cv::Point3f(state_.at<float>(3), state_.at<float>(4), state_.at<float>(5)); }
  cv::Point3f getPredictedAcceleration() const { return cv::Point3f(state_.at<float>(6), state_.at<float>(7), state_.at<float>(8)); }
  
  int getStateSize() const { return state_size_;}

  cv::Mat getErrorCovPre() const  { return kf_.errorCovPre; }
  cv::Mat getErrorCovPost() const { return kf_.errorCovPost; }
  cv::Mat getGain()         const { return kf_.gain;}
  cv::Mat getMeasurement()  const { return kf_.measurementMatrix;}

  // Set and get ErrorEllipse:
  void setErrorEllipseXY(const cv::RotatedRect& error_ellipse) { this->error_ellipse_xy_ = error_ellipse; }
  const cv::RotatedRect& getErrorEllipseXY() const { return this->error_ellipse_xy_; }
  void setErrorEllipseYZ(const cv::RotatedRect& error_ellipse) { this->error_ellipse_yz_ = error_ellipse; }
  const cv::RotatedRect& getErrorEllipseYZ() const { return this->error_ellipse_yz_; }

  // Set and get Covariance and Confidence @TODO
  void setCovariance(double x, double y, double z, double angle_x, double angle_y, double angle_z);
  void getCovariance(double& x, double& y, double& z, double& angle_x, double& angle_y, double& angle_z) const;
  // void setConfidence(double confidence) { this->confidence = confidence; }
  // double getConfidence() const { return this->confidence; }
  
  /**
   * @brief The state_size_ is used to select the correct model. For state_size_==6 constant vel., for state_size==9 no constant vel.
   */
  int state_size_;
  int measurement_size_;
  float covariance_[];

protected:
  /**
   * @brief Initializes member variables and OpenCV Kalman filter attributes
   * @param state_size size of state vector
   * @param measurement_size size of measurement vector
   */

  // with the state_size 6 or 9 a kalman filter model is choosen!!!
  void initKalman(int state_size = 6, int measurement_size = 3);


  // Protected member variables
  cv::KalmanFilter kf_;

  cv::Mat state_;
  cv::Mat measurement_;
  // Input:
  cv::Point3d latest_detected_position_;
  ros::Time   last_observation_time_; 
  
  // Output:
  cv::Point3d position_;
  cv::Point3d velocity_;
  cv::Point3d acceleration_;
  ros::Time   time_stamp_;
  cv::RotatedRect error_ellipse_xy_;
  cv::RotatedRect error_ellipse_yz_;  // @TODO

};
}
#endif
