#include "orca_vision/util.h"
#include "orca_vision/stereo_image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace orca_vision {

//=========================
// Debugging
//=========================

void displayMatches(
  const cv::Mat &l_image, const std::vector<cv::KeyPoint> &l_points,
  const cv::Mat &r_image, const std::vector<cv::KeyPoint> &r_points,
  const std::vector<cv::DMatch> &matches, const std::string window)
{
  cv::Mat out;
  cv::drawMatches(l_image, l_points, r_image, r_points, matches, out,
    cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow(window, out);
  cv::waitKey(1); // Required
}

//=========================
// Image
//=========================

bool Image::initialize(const sensor_msgs::ImageConstPtr &image)
{
  auto image_ptr = cv_bridge::toCvCopy(image, "mono8");

  // CLion isn't happy with cv_bridge types... suppress fake errors
#pragma clang diagnostic push
#pragma ide diagnostic ignored "CannotResolve"
  image_ = image_ptr->image;
  stamp_ = image_ptr->header.stamp;
#pragma clang diagnostic pop

  // Detect features
  auto detector = cv::ORB::create(200); // WTA_K == 2, so BFMatcher should use NORM_HAMMING
  keypoints_.clear();
  detector->detect(image_, keypoints_);

  if (keypoints_.size() < MIN_FEATURES)
  {
    return false;
  }

  // Compute descriptors
  descriptors_.release();
  detector->compute(image_, keypoints_, descriptors_);

  return true;
}

//=========================
// StereoImage
//=========================

bool StereoImage::initialize(
  const image_geometry::StereoCameraModel &camera_model,
  const sensor_msgs::ImageConstPtr &left_image,
  const sensor_msgs::ImageConstPtr &right_image,
  const cv::DescriptorMatcher &matcher)
{
  if (!left_.initialize(left_image) || !right_.initialize(right_image))
  {
    return false;
  }

  // Find matching features
  std::vector<cv::DMatch> candidate_matches;
  matcher.match(left_.descriptors(), right_.descriptors(), candidate_matches, cv::noArray());

  if (candidate_matches.size() < MIN_FEATURES)
  {
    return false;
  }

  // Keep some stats
  std::vector<float> epipolar_errors;
  std::vector<float> disparities;
  std::vector<float> match_distances;

  // Refine the list of matches
  int good_matches = 0;
  matches_.clear();
  matches_3d_.clear();
  for (int i = 0; i < candidate_matches.size(); ++i)
  {
    int left_idx = candidate_matches[i].queryIdx;
    int right_idx = candidate_matches[i].trainIdx;

    float epipolar_error = std::fabs(left_.keypoints()[left_idx].pt.y - right_.keypoints()[right_idx].pt.y);
    float disparity = left_.keypoints()[left_idx].pt.x - right_.keypoints()[right_idx].pt.x;

    if (debug_)
    {
      epipolar_errors.push_back(epipolar_error);
      disparities.push_back(disparity);
      match_distances.push_back(candidate_matches[i].distance);
    }

    if (candidate_matches[i].distance < MAX_DISTANCE && epipolar_error < 10.0 && disparity > 10.0)
    {
      matches_.push_back(candidate_matches[i]);

      // Project into 3D in left_camera_frame
      cv::Point3d p;
      camera_model.projectDisparityTo3d(left_.keypoints()[left_idx].pt, disparity, p);
      matches_3d_.push_back(p);

      ++good_matches;
    }
  }

  if (debug_)
  {
    float median_disparity = destructive_median(disparities);
    float median_epipolar_error = destructive_median(epipolar_errors);
    float median_distance = destructive_median(match_distances);
    ROS_DEBUG("%d good matches, median epipolar error=%g, median disparity=%g, median Z=%g, median match distance %g",
      good_matches, median_epipolar_error, median_disparity, camera_model.getZ(median_disparity), median_distance);
  }

  displayMatches(
    left_.image(),
    left_.keypoints(),
    right_.image(),
    right_.keypoints(),
    matches_, STEREO_WINDOW);

  return good_matches >= MIN_FEATURES;
}

bool StereoImage::computeTransform(
  const StereoImage *key_image,
  const cv::DescriptorMatcher &matcher,
  std::vector<cv::Point3f> &key_good,
  std::vector<cv::Point3f> &curr_good)
{
  key_good.clear();
  curr_good.clear();

  // Find features that match in the key image and the current image
  std::vector<cv::DMatch> candidate_time_matches;
  matcher.match(left_.descriptors(), key_image->left().descriptors(), candidate_time_matches, cv::noArray());

  // Refine the list of matches
  std::vector<cv::DMatch> time_matches;
  for (int i = 0; i < candidate_time_matches.size(); ++i)
  {
    if (candidate_time_matches[i].distance < MAX_DISTANCE)
    {
      time_matches.push_back(candidate_time_matches[i]);
    }
  }

  if (time_matches.size() < MIN_FEATURES)
  {
    return false;
  }

  displayMatches(
    left_.image(),
    left_.keypoints(),
    key_image->left().image(),
    key_image->left().keypoints(),
    time_matches, TIME_WINDOW);

  // Look for features that were found in all 4 images (i.e., appear in all 3 match arrays)
  for (int i = 0; i < time_matches.size(); ++i)
  {
    for (int j = 0; j < matches_.size(); ++j)
    {
      for (int k = 0; k < key_image->matches().size(); ++k)
      {
        if (time_matches[i].queryIdx == matches_[j].queryIdx &&
          time_matches[i].trainIdx == key_image->matches()[k].queryIdx)
        {
          key_good.push_back(key_image->matches_3d()[k]);
          curr_good.push_back(matches_3d_[j]);
        }
      }
    }
  }

  if (curr_good.size() < MIN_FEATURES)
  {
    return false;
  }

  // Compute the change in the camera pose
  tf2::Transform change;
  rigidTransform(curr_good, key_good, change);

  // Update curr_image.odom_to_left_camera_
  tf2::Transform odom_to_left_camera = change * key_image->odomToLeftCamera();
  odom_to_left_camera_ = odom_to_left_camera;

  return true;
}

} // namespace orca_vision