import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
import numpy as np
import simple_kf

# Estimate a local pose

# -- position (hidden)
# -- velocity (hidden)
# -- acceleration (measured)

G = 9.80665     # Gravitational constant
dt = 1. / 50    # TODO get dt from message


class LocalPose(object):

    def __init__(self):
        self.kf = simple_kf.KalmanFilter(state_dim=9, measurement_dim=3)

        # TODO drag ???
        dt2 = 0.5 * dt * dt
        self.kf.F = np.array([
            [1, 0, 0,    dt, 0, 0,    dt2, 0, 0],   # x
            [0, 1, 0,    0, dt, 0,    0, dt2, 0],   # y
            [0, 0, 1,    0, 0, dt,    0, 0, dt2],   # z

            [0, 0, 0,    1, 0, 0,     dt, 0, 0],    # x'
            [0, 0, 0,    0, 1, 0,     0, dt, 0],    # y'
            [0, 0, 0,    0, 0, 1,     0, 0, dt],    # z'

            [0, 0, 0,    0, 0, 0,     1, 0, 0],     # x''
            [0, 0, 0,    0, 0, 0,     0, 1, 0],     # y''
            [0, 0, 0,    0, 0, 0,     0, 0, 1],     # z''
        ])
        print 'F\n', self.kf.F

        # Process noise
        # TODO look at this more carefully
        # Q_var = 0.0001
        # Q = Q_discrete_white_noise(dim=3, dt=dt, var=Q_var)
        self.kf.Q = np.eye(9) * 0.0001
        #self.kf.Q = np.array([[2.5e-05, 5.0e-05, 5.0e-05], [5.0e-05, 1.0e-04, 1.0e-04], [5.0e-05, 1.0e-04, 1.0e-04]])
        print 'Q\n', self.kf.Q

        # Measurement matrix, translates state into measurement space
        self.kf.H = np.array([
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ])

        print 'H\n', self.kf.H

        # Measurement noise
        R_var = 0.003 * 0.003
        self.kf.R = np.eye(3) * R_var
        print 'R\n', self.kf.R

        # Initialize path message
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Set up publications and subscriptions
        self.path_pub = rospy.Publisher('/orca_base/python_estimated', Path, queue_size=1)
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)

    def imu_callback(self, data):
        # TODO rotate lin acc into place

        # Estimate position
        self.kf.predict()
        z = np.array([[data.linear_acceleration.x], [data.linear_acceleration.y], [data.linear_acceleration.z - G]])
        self.kf.update(z)

        # Update path message and publish
        pose = PoseStamped()
        pose.header.stamp = data.header.stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.kf.x[0]
        pose.pose.position.y = self.kf.x[1]
        pose.pose.position.z = self.kf.x[2]
        pose.pose.orientation = data.orientation
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('local_pose', anonymous=True)
    np.set_printoptions(precision=4, suppress=True)
    try:
        estimator = LocalPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
