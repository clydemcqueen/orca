#!/usr/bin/env python

import ms5837
import rospy
from orca_msgs.msg import Barometer
  
def run():
  # Initialize ROS
  rospy.init_node('bar30_node', anonymous=True)
  pub = rospy.Publisher('/barometer', Barometer, queue_size=1)

  # Connect to the Bar30 sensor on I2C bus 1
  sensor = ms5837.MS5837_30BA(1)
  if not sensor.init():
    rospy.logerr("Can't initialize Bar30")
    exit(1)

  sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
  rate = rospy.Rate(50)

  # Loop until Ctrl-C
  while not rospy.is_shutdown():
    if sensor.read():
      msg = Barometer()
      msg.pressure = sensor.pressure()          # mbar
      msg.temperature = sensor.temperature()    # Celsius
      msg.depth = sensor.depth()                # meters
      pub.publish(msg)
    else:
      rospy.logerr("Can't read Bar30")
    rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass