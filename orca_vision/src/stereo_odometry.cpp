#include "orca_vision/util.h"
#include "orca_vision/stereo_image.h"

#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace orca_vision {

//=========================
// Constants
//=========================

constexpr const int QUEUE_SIZE = 30;            // Number of images to keep for approx sync

//=========================
// StereoOdometryNode
//=========================

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;

class StereoOdometryNode
{
  image_geometry::StereoCameraModel camera_model_;      // Stereo camera model (intrinsics, extrinsics)
  StereoImage *prev_image_, *key_image_;                // Stereo images TODO try smart pointers again?
  cv::BFMatcher matcher_{cv::NORM_HAMMING, true};       // Brute-force feature matcher; enable cross-checking

  message_filters::Synchronizer<SyncPolicy> *approx_sync_;                                // Simulate synchronized cameras
  image_transport::SubscriberFilter image_left_sub_, image_right_sub_;                    // Subscribe to left and right monochrome rectified images
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_left_sub_, info_right_sub_;   // Subscribe to left and right camera info messages

  ros::Publisher odom_pub_;                       // Publish odometry message
  ros::Publisher key_features_pub_;               // Publish key cloud
  ros::Publisher curr_features_pub_;              // Publish current cloud
  tf2_ros::TransformBroadcaster tf_broadcaster_;  // Publish tf messages

  tf2::Transform odom_to_base_;           // Last odom => base_link transform
  tf2::Transform base_to_left_camera_;    // Static base_link => left_camera_frame from robot_state_publisher

public:

  //=========================
  // Constructor
  //=========================

  StereoOdometryNode(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, tf2::Transform &odom_to_base, tf2::Transform &base_to_left_camera):
    odom_to_base_{odom_to_base}, base_to_left_camera_{base_to_left_camera}, prev_image_{nullptr}, key_image_{nullptr}
  {
    ros::NodeHandle left_nh(nh, "stereo/left");
    ros::NodeHandle right_nh(nh, "stereo/right");
    ros::NodeHandle left_pnh(nh_priv, "stereo/left");
    ros::NodeHandle right_pnh(nh_priv, "stereo/right");

    image_transport::ImageTransport left_it(left_nh);
    image_transport::ImageTransport right_it(right_nh);
    image_transport::TransportHints hints_left("raw", ros::TransportHints(), left_pnh);
    image_transport::TransportHints hints_right("raw", ros::TransportHints(), right_pnh);

    image_left_sub_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hints_left);
    image_right_sub_.subscribe(right_it, right_nh.resolveName("image_rect"), 1, hints_right);
    info_left_sub_.subscribe(left_nh, "camera_info", 1);
    info_right_sub_.subscribe(right_nh, "camera_info", 1);

    approx_sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(QUEUE_SIZE),
      image_left_sub_, image_right_sub_, info_left_sub_, info_right_sub_);
    approx_sync_->registerCallback(boost::bind(&StereoOdometryNode::imageTransportCallback, this, _1, _2, _3, _4));

    odom_pub_ = nh_priv.advertise<nav_msgs::Odometry>("stereo_odom", 1);
    key_features_pub_ = nh_priv.advertise<sensor_msgs::PointCloud2>("key", 1);
    curr_features_pub_ = nh_priv.advertise<sensor_msgs::PointCloud2>("curr", 1);
  }

  ~StereoOdometryNode()
  {
    delete approx_sync_;
    // TODO delete the stereo images
  }

private:

  //=========================
  // imageTransportCallback
  //=========================

  void imageTransportCallback(
    const sensor_msgs::ImageConstPtr& image_left,
    const sensor_msgs::ImageConstPtr& image_right,
    const sensor_msgs::CameraInfoConstPtr& camera_info_left,
    const sensor_msgs::CameraInfoConstPtr& camera_info_right)
  {
    // Initialize the camera model
    if (!camera_model_.initialized())
    {
      if (!camera_model_.fromCameraInfo(camera_info_left, camera_info_right))
      {
        ROS_ERROR("Can't initialize camera model, quitting");
        exit(-1);
      }
      ROS_INFO("Camera model initialized, baseline %g cm", camera_model_.baseline() * 100);
    }

    // Analyze the stereo image
    StereoImage *curr_image = new StereoImage();
    if (!curr_image->initialize(camera_model_, image_left, image_right, matcher_))
    {
      printf("No stereo features\n");
    }
    else
    {
      if (prev_image_ == nullptr)
      {
        // Bootstrap
        printf("Bootstrap\n");
        curr_image->setOdomToLeftCamera(base_to_left_camera_ * odom_to_base_);
        key_image_ = prev_image_ = curr_image;
      }
      else
      {
        std::vector<cv::Point3f> key_good;
        std::vector<cv::Point3f> curr_good;

        // Compute transform from the key image to the current image
        bool good_odometry = curr_image->computeTransform(key_image_, matcher_, key_good, curr_good);

        if (!good_odometry && key_image_ != prev_image_)
        {
          // Promote the previous image to key image, and try again
          printf("Updating key image\n");
          if (key_image_ != nullptr)
          {
            delete key_image_;
          }
          key_image_ = prev_image_;
          good_odometry = curr_image->computeTransform(key_image_, matcher_, key_good, curr_good);
        }

        if (good_odometry)
        {
          // Success!
          odom_to_base_ =  base_to_left_camera_.inverse() * curr_image->odomToLeftCamera();

          publishFeatures(key_good, true);
          publishFeatures(curr_good, false);

          // Current image becomes previous image
          if (prev_image_ != nullptr && prev_image_ != key_image_)
          {
            delete prev_image_;
          }
          prev_image_ = curr_image;
        }
        else
        {
          // We've lost odometry. Start over.
          printf("Lost odometry\n");

          delete key_image_;
          key_image_ = prev_image_ = nullptr;
        }
      }
    }

    // Always publish odometry
    publishOdometry();
  }

  //=========================
  // publishFeatures
  //=========================

  void publishFeatures(const std::vector<cv::Point3f> &points, bool key)
  {
    // TODO only if there are subscribers

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < points.size(); ++i)
    {
      cloud.push_back(pcl::PointXYZ(points[i].x, points[i].y, points[i].z));
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);

    msg.header.frame_id = "left_camera_frame"; // Fill this in after toROSMsg TODO bug?

    if (key)
    {
      key_features_pub_.publish(msg);
    }
    else
    {
      curr_features_pub_.publish(msg);
    }
  }

  //=========================
  // publishOdometry
  //=========================

  void publishOdometry()
  {
    tf2::Quaternion q = odom_to_base_.getRotation();
    tf2::Vector3 t = odom_to_base_.getOrigin();

    // Publish odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    // TODO covariance; if we have good data, this is low, but if we're dropping images this zooms way up???
    // TODO twist
    odom_pub_.publish(odom_msg);

    // Publish transform
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation = tf2::toMsg(t);
    odom_tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(odom_tf);
  }
};

} // namespace orca_vision

//=========================
// main
//=========================

int main(int argc, char** argv)
{
  // Init ROS
  ros::init(argc, argv, "stereo_odometry");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_priv("~");

  // Init odom => base_link at the origin
  tf2::Transform odom_to_base(tf2::Matrix3x3::getIdentity(), tf2::Vector3(0, 0, 0));
  ROS_INFO("odom => base_link %s", orca_vision::tf2_to_string(odom_to_base).c_str());

  // Another node should be publishing the base_link to left_camera_frame transform
  ROS_INFO("Waiting for transform base_link => left_camera_frame...");
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf(tf_buffer);
  tf2::Transform base_to_left_camera;
  ros::Rate rate(2);
  while (nh.ok())
  {

    if (tf_buffer.canTransform("left_camera_frame", "base_link", ros::Time(0), ros::Duration(0)))
    {
      geometry_msgs::TransformStamped transform_msg = tf_buffer.lookupTransform("left_camera_frame", "base_link", ros::Time(0));
      tf2::fromMsg(transform_msg.transform, base_to_left_camera);
      ROS_INFO("base_link => left_camera_frame %s", orca_vision::tf2_to_string(base_to_left_camera).c_str());
      break;
    }

    rate.sleep();
  }

  ROS_INFO("Initializing node...");
  orca_vision::StereoOdometryNode stereo_odometry_node{nh, nh_priv, odom_to_base, base_to_left_camera};

  cv::namedWindow(orca_vision::TIME_WINDOW, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(orca_vision::STEREO_WINDOW, cv::WINDOW_AUTOSIZE);

  // Spin
  ROS_INFO("Waiting for images...");
  ros::spin();

  return 0;
}
