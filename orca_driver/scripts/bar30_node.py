#!/usr/bin/env python

# This is a simple ROS wrapper for the BlueRobotics Bar30 sensor
# https://github.com/bluerobotics/ms5837-python/blob/master/ms5837.py must be in the Python path

import rospy
import ms5837
from orca_msgs.msg import Barometer
  
def run():
  # Initialize ROS
  rospy.init_node('bar30_node', anonymous=True)
  pub = rospy.Publisher('/barometer', Barometer, queue_size=1)

  # Connect to the Bar30 sensor on I2C bus 1
  rospy.loginfo("Connecting to Bar30...")
  sensor = ms5837.MS5837_30BA(1)
  if not sensor.init():
    rospy.logerr("Can't initialize Bar30")
    exit(1)

  sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
  rate = rospy.Rate(10) # Max rate on the RPi3 seems to be 22 hz

  # Loop until Ctrl-C
  rospy.loginfo("Connected to Bar30, entering main loop")
  while not rospy.is_shutdown():
    if sensor.read():
      msg = Barometer()
      msg.header.stamp = rospy.Time.now()
      msg.pressure = sensor.pressure() * 100.0  # Pascals
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
