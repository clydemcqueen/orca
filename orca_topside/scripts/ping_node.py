#!/usr/bin/env python

# ROS node to publish a regular heartbeat
# If the sub doesn't hear the heartbeat, it will disarm

import rospy
from std_msgs.msg import Empty

def run():
  # Initialize ROS
  rospy.init_node('ping_node', anonymous=True)
  pub = rospy.Publisher('/ping', Empty, queue_size=1)

  rate = rospy.Rate(1) # Publish once per second

  # Loop until Ctrl-C
  rospy.loginfo("Entering main loop")
  while not rospy.is_shutdown():

    msg = Empty()
    pub.publish(msg)

    rate.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass