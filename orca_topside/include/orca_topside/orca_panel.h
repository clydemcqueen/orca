#ifndef ORCA_PANEL_H
#define ORCA_PANEL_H

#include <rviz/panel.h>
#include <ros/node_handle.h>
#include <qt5/QtCore/QtCore>

#include "orca_msgs/Barometer.h"
#include <orca_msgs/Battery.h>
#include <orca_msgs/Control.h>
#include <orca_msgs/Leak.h>
#include <orca_msgs/Proc.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>

class QLineEdit;
class QLabel;

namespace orca_topside {

class OrcaPanel: public rviz::Panel
{
public:
  OrcaPanel(QWidget* parent = 0);
  ~OrcaPanel();

protected:
  ros::NodeHandle nh_;
  ros::Timer timer_;

  QLabel* battery_viewer_;
  QLabel* camera_tilt_viewer_;
  QLabel* depth_viewer_;
  QLabel* leak_viewer_;
  QLabel* lights_viewer_;
  QLabel* mode_viewer_;
  QLabel* proc_viewer_;
  QLabel* temperature_viewer_;
  QLabel* yaw_viewer_;

  QPalette ok_palette_;
  QPalette alert_palette_;
  QPalette danger_palette_;

  ros::Subscriber baro_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber control_sub_;
  ros::Subscriber leak_sub_;
  ros::Subscriber proc_sub_;

  void baroCallback(const orca_msgs::Barometer::ConstPtr &msg);
  void batteryCallback(const orca_msgs::Battery::ConstPtr &msg);
  void controlCallback(const orca_msgs::Control::ConstPtr &msg);
  void leakCallback(const orca_msgs::Leak::ConstPtr &msg);
  void procCallback(const orca_msgs::Proc::ConstPtr &msg);

  void timerCallback(const ros::TimerEvent &event);
};

} // namespace orca_topside

#endif // ORCA_PANEL_H
