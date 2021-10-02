#!/usr/bin/env python

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import rospy
import time
import tf


class Self_balancing_robot():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('controller')  

        self.orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.orientation_euler = [0.0, 0.0, 0.0]

        self.setpoint_euler = 0

        self.cmd_right = Float64()
        self.cmd_right.data = 0
        self.cmd_left = Float64()
        self.cmd_left.data = 0

        # Kp, Kd and ki for [roll, pitch, yaw].
        self.Kp = 1500
        self.Ki = 1
        self.Kd = 70

        self.error_sum = 0
        self.error_change = 0
        self.prev_error = 0

        # # This is the sample time needed to run pid.
        self.sample_time = 0.060

        self.right_wheel_pub = rospy.Publisher('/Rev4_position_controller/command', Float64, queue_size=1)
        self.left_wheel_pub = rospy.Publisher('/Rev5_position_controller/command', Float64, queue_size=1)

        # Subscribing to /drone_command, imu/data
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
    # Imu callback function
    def imu_callback(self, msg):

        self.orientation_quaternion[0] = msg.orientation.x
        self.orientation_quaternion[1] = msg.orientation.y
        self.orientation_quaternion[2] = msg.orientation.z
        self.orientation_quaternion[3] = msg.orientation.w

    # PID finction
    def pid(self):

        # Converting quaternion to euler angles
        self.orientation_euler = tf.transformations.euler_from_quaternion(self.orientation_quaternion)

        error = 0
        pid_output = 0

        error = self.setpoint_euler - self.orientation_euler[0]
        self.error_sum += error
        self.error_change = error - self.prev_error
        pid_output = self.Kp * error + self.Kd * self.error_change + self.Ki * self.error_sum

        self.cmd_right.data = pid_output
        self.cmd_left.data = -pid_output

        self.prev_error = error
        
        self.right_wheel_pub.publish(self.cmd_right)
        self.left_wheel_pub.publish(self.cmd_left)

if __name__ == '__main__':
    rob = Self_balancing_robot()
    r = rospy.Rate(1/rob.sample_time)  
    while not rospy.is_shutdown():
        try:
            rob.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass