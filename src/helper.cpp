#include <simple_kf/helper.h>


namespace simple_kf
{
cv::RotatedRect get2DErrorEllipse(const cv::Point2f& mean, const cv::Mat& cov, double chisq)
{
  try
  {
    // get the eigenvalues and eigenvectors
    cv::Mat eigen_values, eigen_vectors;
    cv::eigen(cov, eigen_values, eigen_vectors); //caution eigen_values will be nan if cov is nan

    float angle = atan2(eigen_vectors.at<float>(0, 1), eigen_vectors.at<float>(0, 0));

    // shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
      angle += 2.0*M_PI;

    // convert to degrees instead of radians
    angle = 180.0 * angle/M_PI;

    // calculate the size of the minor and major axes
    float half_major_axis_size = chisq*sqrt(eigen_values.at<float>(0));
    float half_minor_axis_size = chisq*sqrt(eigen_values.at<float>(1));

    // return the oriented ellipse
    // the -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    return cv::RotatedRect(mean, cv::Size2f(half_minor_axis_size, half_major_axis_size), -angle);
  }
  catch (const cv::Exception& e)
  {
    ROS_ERROR("Exception caught in get2DErrorEllipse:\n%s", e.what());
    return cv::RotatedRect(mean, cv::Size2f(0.0f, 0.0f), 0.0f);
  }
}
}