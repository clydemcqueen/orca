#ifndef VISION_UTIL_H
#define VISION_UTIL_H

#include <opencv2/core/mat.hpp>
#include <tf2/LinearMath/Transform.h>

namespace orca_vision {

//=========================
// std::
//=========================

template <typename T>
T destructive_median(std::vector<T> &v)
{
  if (v.size() == 0)
  {
    return 0;
  }
  else
  {
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }
}

//=========================
// cv:: <==> tf2::
//=========================

// cv::Mats must be CV_32F
// cv::Mats that represent vectors must be 1x3

void cv_to_tf2(const cv::Mat &m, tf2::Vector3 &t);
void cv_to_tf2(const cv::Mat &m, tf2::Matrix3x3 &r);

void tf2_to_cv(const tf2::Vector3 &t, cv::Mat &m);
void tf2_to_cv(const tf2::Matrix3x3 &r, cv::Mat &m);

std::string tf2_to_string(const tf2::Vector3 &v);
std::string tf2_to_string(const tf2::Matrix3x3 &r);
std::string tf2_to_string(const tf2::Transform &t);

cv::Point3f toPoint3f(const tf2::Vector3 &p);
tf2::Vector3 toVector3(const cv::Point3f &p);

void testConversions();

//=========================
// Axis-angle calculations
//=========================

float rotationAngle(const cv::Mat &r);
float rotationAngle(const tf2::Matrix3x3 &r);

//=========================
// Rigid transforms
//=========================

// Use SVD to compute a rigid transform (R, t) such that B = R * A + t
// A and B must be Nx3
// cv::Mats must be CV_32F
// R will be 3x3
// t will be 1x3
void rigidTransform(const cv::Mat &A, const cv::Mat &B, cv::Mat &R, cv::Mat &t);
void rigidTransform(std::vector<cv::Point3f> &A, std::vector<cv::Point3f> &B, tf2::Transform &transform);

void testRigidTransform();

} // namespace orca_vision

#endif // VISION_UTIL_H
