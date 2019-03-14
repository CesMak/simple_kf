#ifndef HELPER_H__
#define HELPER_H__

#include <iostream>
#include <string>

// all in includes here:
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <alfons_msgs/KfState.h>



// macro to convert a string to a double
#define SSTR(x) static_cast<std::ostringstream&>(std::ostringstream() << std::dec << x).str()

//macro to round a value to 2 digits
#define ROUND2(x) std::round(x * 100) / 100
#define ROUND3(x) std::round(x * 1000) / 1000



namespace simple_kf
{
    cv::RotatedRect get2DErrorEllipse(const cv::Point2f& mean, const cv::Mat& cov, double chisq = 5.991);
}

#endif