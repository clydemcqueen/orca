#!/usr/bin/env python

# ROS node to publish processor status

import os
import rospy
from orca_msgs.msg import Proc
  
def run():
  # Initialize ROS
  rospy.init_node('proc_node', anonymous=True)
  pub = rospy.Publisher('/proc', Proc, queue_size=1)

  rate = rospy.Rate(1) # Publish once per second

  # Loop until Ctrl-C
  rospy.loginfo("Entering main loop")
  while not rospy.is_shutdown():

    # Query the Raspberry Pi 3 CPU temp, result is given in degrees C * 1000
    # When temp hits 85 degrees C the CPU will throttle
    temp = os.popen("cat /sys/class/thermal/thermal_zone0/temp").read().strip()

    msg = Proc()
    msg.header.stamp = rospy.Time.now()
    msg.cpu_temp = float(temp) / 1000
    pub.publish(msg)

    rate.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
