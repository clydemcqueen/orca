#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <image_transport/transport_hints.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

// TODO why only getting 19% odometry? What's not working?
// TODO measure performance -- are we getting behind?
// TODO assert scale
// TODO repeat velocity if we drop a frame

//=========================
// Constants
//=========================

const cv::TermCriteria TERM_CRIT(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
const cv::Size SUB_PIX_WIN_SIZE(10,10), win_size(31,31);
constexpr const int MAX_FEATURES =      500;  // Max features we'll track
constexpr const int MIN_FEATURES =      200;  // Min features we'll track
constexpr const int MIN_ODOM =           50;  // Min features to compute odometry
constexpr const int MIN_ODOM_INLIERS =   20;  // Min inliers to use odometry results

constexpr const int QUEUE_SIZE = 30;

// Mask out part of the image
constexpr const int LR_BUFFER = 0;    // 300 for Orca front-facing video
constexpr const int TOP_BUFFER = 0;   // 160 for Orca front-facing video

//=========================
// Globals
//=========================

int g_frames = 0;           // Total number of frames
int g_frames_dropped = 0;   // Frames dropped
int g_find = 0;             // Calls to findFeatures
int g_good_odom = 0;        // Good odometry results

//=========================
// cv:: to tf::
//=========================

void matToMatrix3x3(cv::Mat &m, tf2::Matrix3x3 &r)
{
  // Must be a 3x3 rotation matrix
  assert(m.dims == 2 && m.size[0] == 3 && m.size[1] == 3);

  for (int i = 0; i < 3; i++)
  {
    r[i].setX(m.at<double>(0, i));
    r[i].setY(m.at<double>(1, i));
    r[i].setZ(m.at<double>(2, i));
  }
}

void matToVector3(cv::Mat &m, tf2::Vector3 &t)
{
  // Must be a 3x1 translation matrix
  assert(m.dims == 2 && m.size[0] == 3 && m.size[1] == 1);

  t.setX(m.at<double>(0, 0));
  t.setY(m.at<double>(1, 0));
  t.setZ(m.at<double>(2, 0));
}

//=========================
// Functions
//=========================

void findFeatures(cv::Mat &image, std::vector<cv::Point2f> &points, cv::Mat &mask)
{
  g_find++;

  // Find Shi-Tomasi features
  goodFeaturesToTrack(image, points, MAX_FEATURES, 0.01, 10, mask, 3, 3, 0, 0.04);

  // Compute subpixel locations
  cornerSubPix(image, points, SUB_PIX_WIN_SIZE, cv::Size(-1,-1), TERM_CRIT);

  // If we didn't find enough features, drop this frame
  if (points.size() < MIN_FEATURES)
  {
    g_frames_dropped++;
    points.clear();

    //std::string message = "DROPPING FRAME with " + std::to_string(points.size()) + " features";
    //std::cout << message << std::endl;
  }
}

void report()
{
  std::cout << "Captured " << std::to_string(g_frames) <<
    "; reset " << std::to_string(g_find) << " (" << std::to_string(g_find * 100 / g_frames) <<
    "%); dropped " << std::to_string(g_frames_dropped) << " (" << std::to_string(g_frames_dropped * 100 / g_frames) <<
    "%); good odom " << std::to_string(g_good_odom) << " (" << std::to_string(g_good_odom * 100 / g_frames) <<
    "%)." << std::endl;
}

//=========================
// ROS node
//=========================

class MonoOdometryNode
{
public:

  MonoOdometryNode(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
  {
    ros::NodeHandle left_nh(nh, "stereo/left");
    ros::NodeHandle left_pnh(nh_priv, "stereo/left");
    
    image_transport::ImageTransport left_it(left_nh);
    image_transport::TransportHints hints_left("raw", ros::TransportHints(), left_pnh);

    image_rect_left_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hints_left);
    camera_info_left_.subscribe(left_nh, "camera_info", 1);

    approx_sync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(QUEUE_SIZE), image_rect_left_, camera_info_left_);
    approx_sync_->registerCallback(boost::bind(&MonoOdometryNode::imageTransportCallback, this, _1, _2));

    position_ = {0, 0, 0};
    orientation_ = tf2::Matrix3x3::getIdentity();
  }

  ~MonoOdometryNode()
  {
    delete approx_sync_;
  }

  void imageTransportCallback(
    const sensor_msgs::ImageConstPtr& image_rect_left,
    const sensor_msgs::CameraInfoConstPtr& camera_info_left)
  {
    g_frames++;

    // Convert image from ROS to OpenCV3
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_rect_left, "mono8");

    // Copy to our instance variable
    image_ptr->image.copyTo(curr_frame_);

    // Create a color image for debugging output
    cv::Mat debug;
    cvtColor(curr_frame_, debug, cv::COLOR_GRAY2BGR);

    // Mask out the unimportant bits
    if (frame_mask_.empty())
    {
      frame_mask_rect_ = cv::Rect(LR_BUFFER, 0, curr_frame_.cols - 2 * LR_BUFFER, curr_frame_.rows - TOP_BUFFER);
      frame_mask_ = cv::Mat::zeros(curr_frame_.rows, curr_frame_.cols, CV_8UC1);
      frame_mask_(frame_mask_rect_) = 1;
      ROS_INFO("Rows %d, cols %d", curr_frame_.rows, curr_frame_.cols);
    }

    // Draw the mask for debugging
    cv::rectangle(debug, frame_mask_rect_, cv::Scalar(0, 0, 0));

    if (focal_length_ == 0.0)
    {
      focal_length_ = (camera_info_left->P[0] + camera_info_left->P[5]) / 2;
      principal_point_ = {camera_info_left->P[2], camera_info_left->P[6]};
      ROS_INFO("Focal length %g, principle point {%g, %g}", focal_length_, principal_point_.x, principal_point_.y);
    }

    if (prev_points_.empty())
    {
      // Bootstrap, or restart after a dropped frame
      findFeatures(curr_frame_, curr_points_, frame_mask_);
    }
    else
    {
      std::vector<uchar> tracking_status;
      std::vector<float> tracking_error;

      // Find the previous features on this frame, and compute optical flow
      cv::calcOpticalFlowPyrLK(prev_frame_, curr_frame_, prev_points_, curr_points_, tracking_status, tracking_error, win_size, 3, TERM_CRIT, 0, 0.001);

      // Eliminate features that we're no longer tracking by compressing the arrays
      size_t new_size = 0;
      for (size_t i = 0; i < curr_points_.size(); i++)
      {
        if (tracking_status[i] && frame_mask_rect_.contains(curr_points_[i]))
        {
          // Draw the feature for debugging
          cv::circle(debug, curr_points_[i], 3, cv::Scalar(0, 0, 255), -1, 8);

          // Compress the arrays
          prev_points_[new_size] = prev_points_[i];
          curr_points_[new_size] = curr_points_[i];

          new_size++;
        }
      }
      prev_points_.resize(new_size);
      curr_points_.resize(new_size);

      // Calculate odometry if we have enough features
      if (curr_points_.size() > MIN_ODOM)
      {
        cv::Mat odom_status, rotation, translation;

        // Compute the Essential Matrix, which includes R and t
        cv::Mat essential_matrix = cv::findEssentialMat(prev_points_, curr_points_, focal_length_, principal_point_,
          cv::RANSAC, 0.999, 4.0, odom_status);

        // Pull R and t out of the Essential Matrix
        int num_inliers = cv::recoverPose(essential_matrix, prev_points_, curr_points_, rotation, translation,
          focal_length_, principal_point_, odom_status);

        if (num_inliers >= MIN_ODOM_INLIERS)
        {
          g_good_odom++;

          // odom_status is of the form Mat(points.size(), 1, CV_8U)
          for (int i = 0; i < odom_status.size[0]; i++)
          {
            // Draw inliers for debugging (any non-zero value is an inlier)
            if (odom_status.at<uchar>(i, 0))
            {
              cv::circle(debug, curr_points_[i], 9, cv::Scalar(0, 255, 0), 2);
            }
          }

          mergeOdom(rotation, translation);
          publishOdom();
        }
      }

      // If we're low on features, find some more
      if (curr_points_.size() < MIN_FEATURES)
      {
        findFeatures(curr_frame_, curr_points_, frame_mask_);
      }
    }

    // Display the debugging image (must call waitkey!)
    cv::imshow("odometry", debug);
    cv::waitKey(1);

    // Swap
    cv::swap(prev_frame_, curr_frame_);
    std::swap(prev_points_, curr_points_);

    if (g_frames % 100 == 0)
    {
      report();
    }
  }

  void mergeOdom(cv::Mat &rotation, cv::Mat &translation)
  {
    tf2::Matrix3x3 r;
    tf2::Vector3 t;

    matToMatrix3x3(rotation, r);
    matToVector3(translation, t);

    orientation_ *= r;
    position_ += t;
  }

  void publishOdom()
  {
    tf2::Quaternion q;
    orientation_.getRotation(q);

    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation = tf2::toMsg(position_);
    odom_tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(odom_tf);
  }

private:

  double focal_length_{0.0};
  cv::Point2d principal_point_;

  cv::Mat curr_frame_, prev_frame_;                     // Copy of current and previous frames
  std::vector<cv::Point2f> curr_points_, prev_points_;  // Current and previous list of features
  cv::Rect frame_mask_rect_;                            // Usable part of the frame
  cv::Mat frame_mask_;                                  // Usable part of the frame

  tf2::Vector3 position_;
  tf2::Matrix3x3 orientation_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Image transport will sync messages for us
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
  image_transport::SubscriberFilter image_rect_left_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_left_;
  message_filters::Synchronizer<MyApproxSyncPolicy> *approx_sync_;
};

//=========================
// Main
//=========================

int main(int argc, char** argv)
{
  // Hack to redirect annoying ffmpeg error "Invalid UE golomb code" into a file, along with all other errors
  // freopen("error.txt", "w", stderr);

  // Create a CV window for debugging
  cv::namedWindow("odometry", cv::WINDOW_AUTOSIZE);

  // Init ROS
  ros::init(argc, argv, "mono_odometry");
  ros::NodeHandle nh{""};
  ros::NodeHandle nh_priv{"~"};
  MonoOdometryNode mono_odometry_node{nh, nh_priv};

  // Spin
  ROS_INFO("We're running, waiting for images to process");
  ros::spin();

  return 0;
}
