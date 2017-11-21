#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <QLineEdit>
#include <QHBoxLayout>
#include <QLabel>

#include "orca_topside/orca_panel.h"

namespace orca_topside {

OrcaPanel::OrcaPanel(QWidget* parent) : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;
  QFont big;
  big.setPointSize(36);

  layout->addWidget(new QLabel("Yaw:"));
  yaw_viewer_ = new QLabel("?°");
  yaw_viewer_->setFont(big);
  layout->addWidget(yaw_viewer_);

  yaw_setpoint_viewer_ = new QLabel("Manual yaw");
  yaw_setpoint_viewer_->setEnabled(false);
  layout->addWidget(yaw_setpoint_viewer_);

  layout->addWidget(new QLabel("Depth:"));
  depth_viewer_ = new QLabel("?m");
  depth_viewer_->setFont(big);
  layout->addWidget(depth_viewer_);

  depth_setpoint_viewer_ = new QLabel("Manual depth");
  depth_setpoint_viewer_->setEnabled(false);
  layout->addWidget(depth_setpoint_viewer_);

  battery_viewer_ = new QLabel("Battery ?V");
  layout->addWidget(battery_viewer_);

  temperature_viewer_ = new QLabel("Water temp ?°");
  layout->addWidget(temperature_viewer_);

  proc_viewer_ = new QLabel("Processor temp ?°");
  layout->addWidget(proc_viewer_);

  camera_tilt_viewer_ = new QLabel("Camera tilt ?°");
  layout->addWidget(camera_tilt_viewer_);

  leak_viewer_ = new QLabel("? leak");
  layout->addWidget(leak_viewer_);

  lights_viewer_ = new QLabel("Lights ?\%");
  layout->addWidget(lights_viewer_);

  layout->addStretch();
  setLayout(layout);

  // Might be nice to pull topic strings from rviz config, but editing rviz config by hand isn't much fun
  baro_sub_ = nh_.subscribe<orca_msgs::Barometer>("/barometer", 10, &OrcaPanel::baroCallback, this);
  battery_sub_ = nh_.subscribe<orca_msgs::Battery>("/orca_driver/battery", 10, &OrcaPanel::batteryCallback, this);
  camera_tilt_sub_ = nh_.subscribe<orca_msgs::Camera>("/orca_base/camera_tilt", 10, &OrcaPanel::cameraTiltCallback, this);
  depth_pid_enable_sub_ = nh_.subscribe<std_msgs::Bool>("/depth_pid/pid_enable", 10, &OrcaPanel::depthPidEnableCallback, this);
  depth_setpoint_sub_ = nh_.subscribe<std_msgs::Float64>("/depth_pid/setpoint", 10, &OrcaPanel::depthSetpointCallback, this);
  leak_sub_ = nh_.subscribe<orca_msgs::Leak>("/orca_driver/leak", 10, &OrcaPanel::leakCallback, this);
  lights_sub_ = nh_.subscribe<orca_msgs::Lights>("/orca_base/lights", 10, &OrcaPanel::lightsCallback, this);
  yaw_pid_enable_sub_ = nh_.subscribe<std_msgs::Bool>("/yaw_pid/pid_enable", 10, &OrcaPanel::yawPidEnableCallback, this);
  yaw_setpoint_sub_ = nh_.subscribe<std_msgs::Float64>("/yaw_pid/setpoint", 10, &OrcaPanel::yawSetpointCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), &OrcaPanel::timerCallback, this);
}

void OrcaPanel::baroCallback(const orca_msgs::Barometer::ConstPtr &msg)
{
  depth_viewer_->setText(QString("%1m").arg(msg->depth, -1, 'f', 2));
  temperature_viewer_->setText(QString("Water temp %1°").arg(msg->temperature));
}

void OrcaPanel::batteryCallback(const orca_msgs::Battery::ConstPtr &msg)
{
  battery_viewer_->setText(QString("Battery %1V").arg(msg->voltage, -1, 'f', 2));
}

void OrcaPanel::cameraTiltCallback(const orca_msgs::Camera::ConstPtr &msg)
{
  ROS_INFO("Tilt %d", msg->tilt);
  camera_tilt_viewer_->setText(QString("Camera tilt %1°").arg(msg->tilt));
}

void OrcaPanel::depthPidEnableCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (!msg->data)
  {
    depth_setpoint_viewer_->setText("Manual depth");
  }
  depth_setpoint_viewer_->setEnabled(msg->data);
}

void OrcaPanel::depthSetpointCallback(const std_msgs::Float64::ConstPtr &msg)
{
  depth_setpoint_viewer_->setText(QString("Depth setpoint %1m").arg(msg->data, -1, 'f', 2));
}

void OrcaPanel::leakCallback(const orca_msgs::Leak::ConstPtr &msg)
{
  // TODO make leaks super obvious
  leak_viewer_->setText(QString(msg->leak_detected ? "LEAK LEAK LEAK" : "No leak"));
}

void OrcaPanel::lightsCallback(const orca_msgs::Lights::ConstPtr &msg)
{
  ROS_INFO("Brightness %d", msg->brightness);
  lights_viewer_->setText(QString("Lights %1\%").arg(msg->brightness));
}

void OrcaPanel::yawPidEnableCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (!msg->data)
  {
    yaw_setpoint_viewer_->setText("Manual yaw");
  }
  yaw_setpoint_viewer_->setEnabled(msg->data);
}

void OrcaPanel::yawSetpointCallback(const std_msgs::Float64::ConstPtr &msg)
{
  yaw_setpoint_viewer_->setText(QString("Yaw setpoint %1°").arg(qRadiansToDegrees(msg->data), -1, 'f', 0));
}

void OrcaPanel::timerCallback(const ros::TimerEvent &event)
{
  tf::TransformListener* tfListener = vis_manager_->getTFClient();
  if (tfListener != nullptr && tfListener->canTransform("base_link", "odom", ros::Time(0)))
  {
    try
    {
      tf::StampedTransform transform;
      tfListener->lookupTransform("base_link", "odom", ros::Time(0), transform);
      tf::Quaternion orientation = transform.getRotation();
      double roll, pitch, yaw;
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw); // TODO why does yaw run cw? Should be ccw
      yaw_viewer_->setText(QString("%1°").arg(qRadiansToDegrees(yaw), -1, 'f', 0));
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR("Exception trying to get odom to base_link transform: %s", ex.what());
    }
  }
  else
  {
    ROS_WARN("Can't transfom odom to base_link");
  }
}

} // orca_topside

// Tell pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orca_topside::OrcaPanel, rviz::Panel)
