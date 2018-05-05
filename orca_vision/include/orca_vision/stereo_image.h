#ifndef STEREO_IMAGE_H
#define STEREO_IMAGE_H

#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Transform.h>

namespace orca_vision {

constexpr const int MIN_FEATURES =  20;      // Min features to pass each stage
constexpr const float MAX_DISTANCE = 25;     // Maximum distance (in matcher space) that we'll tolerate

//=========================
// Debugging
//=========================

const std::string TIME_WINDOW = "curr left and key left";       // Debug window 1 name
const std::string STEREO_WINDOW = "curr left and curr right";   // Debug window 2 name

void displayMatches(
  const cv::Mat &l_image, const std::vector<cv::KeyPoint> &l_points,
  const cv::Mat &r_image, const std::vector<cv::KeyPoint> &r_points,
  const std::vector<cv::DMatch> &matches, const std::string window);

//=========================
// Image
//=========================

class Image
{
  cv::Mat image_;                         // Image data
  ros::Time stamp_;                       // Image time
  std::vector<cv::KeyPoint> keypoints_;   // Feature locations
  cv::Mat descriptors_;                   // Feature descriptions

public:

  // Initialize
  bool initialize(const sensor_msgs::ImageConstPtr &image);

  // Getters
  const ros::Time &stamp() const { return stamp_; }
  const cv::Mat &image() const { return image_; }
  const std::vector<cv::KeyPoint> &keypoints() const { return keypoints_; }
  const cv::Mat &descriptors() const { return descriptors_; }
};

//=========================
// StereoImage
//=========================

class StereoImage
{
  Image left_, right_;                    // Left and right image data
  std::vector<cv::DMatch> matches_;       // List of features found in both left and right
  std::vector<cv::Point3f> matches_3d_;   // Feature locations projected into 3D
  tf2::Transform odom_to_left_camera_;    // Transform odom => left_camera_frame
  bool debug_;

public:

  StereoImage(bool debug=false): odom_to_left_camera_{tf2::Matrix3x3::getIdentity(), tf2::Vector3()}, debug_{debug} {}

  // Initialize
  bool initialize(
    const image_geometry::StereoCameraModel &camera_model,
    const sensor_msgs::ImageConstPtr &left_image,
    const sensor_msgs::ImageConstPtr &right_image,
    const cv::DescriptorMatcher &matcher);

  // Getters
  const Image &left() const { return left_; }
  const Image &right() const { return right_; }
  const std::vector<cv::DMatch> &matches() const { return matches_; }
  const std::vector<cv::Point3f> &matches_3d() const { return matches_3d_; }
  const tf2::Transform &odomToLeftCamera() const { return odom_to_left_camera_; }

  // Compute odom_to_left_camera_
  bool computeTransform(
    const StereoImage *key_image,
    const cv::DescriptorMatcher &matcher,
    std::vector<cv::Point3f> &key_good,
    std::vector<cv::Point3f> &curr_good);

  // Set odom_to_left_camera_
  void setOdomToLeftCamera(const tf2::Transform &odom_to_left_camera) { odom_to_left_camera_ = odom_to_left_camera; }
};

} // namespace orca_vision

#endif // STEREO_IMAGE_H