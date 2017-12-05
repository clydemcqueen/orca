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

OrcaPanel::OrcaPanel(QWidget* parent) : rviz::Panel(parent),
  depth_pid_enabled_{false},
  yaw_pid_enabled_{false},
  depth_setpoint_{0},
  yaw_setpoint_{0}
{
  ROS_DEBUG("Constructing OrcaPanel");

  ok_palette_ = palette();
  alert_palette_ = palette();
  alert_palette_.setColor(QPalette::Background, Qt::yellow);
  danger_palette_  = palette();
  danger_palette_.setColor(QPalette::Background, Qt::red);
  danger_palette_.setColor(QPalette::Foreground, Qt::white);

  QVBoxLayout* layout = new QVBoxLayout;
  QFont big;
  big.setPointSize(16);

  mode_viewer_ = new QLabel("Mode unknown");
  layout->addWidget(mode_viewer_);

  yaw_viewer_ = new QLabel("Heading unknown");
  yaw_viewer_->setFont(big);
  layout->addWidget(yaw_viewer_);

  depth_viewer_ = new QLabel("Depth unknown");
  depth_viewer_->setFont(big);
  layout->addWidget(depth_viewer_);

  battery_viewer_ = new QLabel("Battery unknown");
  battery_viewer_->setFont(big);
  battery_viewer_->setAutoFillBackground(true);
  battery_viewer_->setPalette(ok_palette_);
  layout->addWidget(battery_viewer_);

  leak_viewer_ = new QLabel("Leak unknown");
  leak_viewer_->setAutoFillBackground(true);
  leak_viewer_->setPalette(ok_palette_);
  layout->addWidget(leak_viewer_);

  proc_viewer_ = new QLabel("Processor temp unknown");
  proc_viewer_->setAutoFillBackground(true);
  proc_viewer_->setPalette(ok_palette_);
  layout->addWidget(proc_viewer_);

  camera_tilt_viewer_ = new QLabel("Camera tilt unknown");
  layout->addWidget(camera_tilt_viewer_);

  lights_viewer_ = new QLabel("Lights unknown");
  layout->addWidget(lights_viewer_);

  temperature_viewer_ = new QLabel("Water temp unknown");
  layout->addWidget(temperature_viewer_);

  layout->addStretch();
  setLayout(layout);

  // Might be nice to pull topic strings from rviz config, but editing rviz config by hand isn't much fun
  baro_sub_ = nh_.subscribe<orca_msgs::Barometer>("/barometer", 10, &OrcaPanel::baroCallback, this);
  battery_sub_ = nh_.subscribe<orca_msgs::Battery>("/orca_driver/battery", 10, &OrcaPanel::batteryCallback, this);
  control_sub_ = nh_.subscribe<orca_msgs::Control>("/orca_base/control", 10, &OrcaPanel::controlCallback, this);
  depth_pid_enable_sub_ = nh_.subscribe<std_msgs::Bool>("/depth_pid/pid_enable", 10, &OrcaPanel::depthPidEnableCallback, this);
  depth_setpoint_sub_ = nh_.subscribe<std_msgs::Float64>("/depth_pid/setpoint", 10, &OrcaPanel::depthSetpointCallback, this);
  leak_sub_ = nh_.subscribe<orca_msgs::Leak>("/orca_driver/leak", 10, &OrcaPanel::leakCallback, this);
  proc_sub_ = nh_.subscribe<orca_msgs::Proc>("/proc", 10, &OrcaPanel::procCallback, this);
  yaw_pid_enable_sub_ = nh_.subscribe<std_msgs::Bool>("/yaw_pid/pid_enable", 10, &OrcaPanel::yawPidEnableCallback, this);
  yaw_setpoint_sub_ = nh_.subscribe<std_msgs::Float64>("/yaw_pid/setpoint", 10, &OrcaPanel::yawSetpointCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), &OrcaPanel::timerCallback, this);

  ROS_DEBUG("OrcaPanel running");
}

OrcaPanel::~OrcaPanel()
{
  ROS_DEBUG("Destructing OrcaPanel");
}

void OrcaPanel::baroCallback(const orca_msgs::Barometer::ConstPtr &msg)
{
  QString depth_string = depth_pid_enabled_ ?
    QString("Depth %1m (%2)").arg(msg->depth, -1, 'f', 2).arg(depth_setpoint_, -1, 'f', 2) :
    QString("Depth %1m").arg(msg->depth, -1, 'f', 2);

  depth_viewer_->setText(depth_string);
  temperature_viewer_->setText(QString("Water temp %1°").arg(msg->temperature));
}

void OrcaPanel::batteryCallback(const orca_msgs::Battery::ConstPtr &msg)
{
  battery_viewer_->setText(QString("Battery %1V").arg(msg->voltage, -1, 'f', 2));
  battery_viewer_->setPalette(msg->voltage > 15 ? ok_palette_ : (msg->voltage > 14 ? alert_palette_ : danger_palette_));
}

void OrcaPanel::controlCallback(const orca_msgs::Control::ConstPtr &msg)
{
  camera_tilt_viewer_->setText(QString("Camera tilt %1°").arg(msg->camera_tilt));
  lights_viewer_->setText(QString("Lights %1\%").arg(msg->brightness));

  switch (msg->mode)
  {
    case orca_msgs::Control::disarmed:
      mode_viewer_->setText("Disarmed");
      break;
    case orca_msgs::Control::manual:
      mode_viewer_->setText("Manual control");
      break;
    case orca_msgs::Control::hold_h:
      mode_viewer_->setText("Hold heading");
      break;
    case orca_msgs::Control::hold_d:
      mode_viewer_->setText("Hold depth");
      break;
    case orca_msgs::Control::hold_hd:
      mode_viewer_->setText("Hold heading and depth");
      break;
    default:
      mode_viewer_->setText("ERROR: unknown mode");
      break;
  }
}

void OrcaPanel::depthPidEnableCallback(const std_msgs::Bool::ConstPtr &msg)
{
  depth_pid_enabled_ = msg->data;
}

void OrcaPanel::depthSetpointCallback(const std_msgs::Float64::ConstPtr &msg)
{
  depth_setpoint_ = msg->data;
}

void OrcaPanel::leakCallback(const orca_msgs::Leak::ConstPtr &msg)
{
  leak_viewer_->setText(QString(msg->leak_detected ? "LEAK LEAK LEAK LEAK" : "No leak"));
  leak_viewer_->setPalette(msg->leak_detected ? danger_palette_ : ok_palette_);
}

void OrcaPanel::procCallback(const orca_msgs::Proc::ConstPtr &msg)
{
  proc_viewer_->setText(QString("Processor temp %1°").arg(msg->cpu_temp, -1, 'f', 1));
  proc_viewer_->setPalette(msg->cpu_temp < 50 ? ok_palette_ : (msg->cpu_temp < 80 ? alert_palette_ : danger_palette_));
}

void OrcaPanel::yawPidEnableCallback(const std_msgs::Bool::ConstPtr &msg)
{
  yaw_pid_enabled_ = msg->data;
}

void OrcaPanel::yawSetpointCallback(const std_msgs::Float64::ConstPtr &msg)
{
  yaw_setpoint_ = -qRadiansToDegrees(msg->data); // TODO sign is flipped
}

void OrcaPanel::timerCallback(const ros::TimerEvent &event)
{
  tf::TransformListener* tfListener = vis_manager_->getTFClient();
  if (tfListener != nullptr && tfListener->canTransform("odom", "base_link", ros::Time(0)))
  {
    try
    {
      tf::StampedTransform transform;
      tfListener->lookupTransform("base_link", "odom", ros::Time(0), transform);
      tf::Quaternion orientation = transform.getRotation();
      double roll, pitch, yaw;
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

      QString heading_string = yaw_pid_enabled_ ?
        QString("Heading %1° (%2)").arg(qRadiansToDegrees(yaw), -1, 'f', 0).arg(yaw_setpoint_, -1, 'f', 0) :
        QString("Heading %1°").arg(qRadiansToDegrees(yaw), -1, 'f', 0);

      yaw_viewer_->setText(heading_string);
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR("Exception trying to get odom to base_link transform: %s", ex.what());
    }
  }
}

} // orca_topside

// Tell pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orca_topside::OrcaPanel, rviz::Panel)
