#!/usr/bin/env python

# Michael DeFilippo, 2022-03-07
'''
Node that subscribes to /cmd_vel and path to goal to get PID modified linear.x and angular.z 
velocity commands to next waypoint on path. Will republish updated velocity command on 
/cmd_vel/pid 
'''

import rospy
import math
import geometry_msgs.msg import Twist
import sensor_msgs.msg import Imu  # Might not use Imu since I have pose data from the localization EKF ***
import nav_msgs.msg import Odometry, Plan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from dynamic_reconfigure.server import Server
from rbot280.config import Cmd_Vel_PID


class Cmd_Vel_PID(object):
    linear_kp = 1  # linear proportional constant
    linear_ki = 1  # linear integral constant
    linear_kd = 1  # linear derivative constant
    angular_kp = 1  # angular proportional constant
    angular_ki = 1  # angular integral constant
    angular_kd = 1  # angular derivative constant
    angular_z, linear_x = 0, 0  # value from cmd_vel
    odom_x, odom_y, odom_yaw = 0, 0, 0  # robot pose and position
    goal_x, goal_y, goal_yaw = 0, 0, 0  # goal pose and position

    # *************** Review thresholds for PID *************************************
    linear_threshold = 5.0
    linear_velocity_threshold = 2
    angular_velocity_threshold = 1.5

    def __init__(self):
        # init rospy
        rospy.init_node('cmd_vel_pid', anonymous=True)
        r = rospy.Rate(2)
        # Subscribers
        rospy.Subscriber("cmd_vel", Twist, callback=self.cmd_vel_cb, queue_size=10)

        # Publishers
        cmd_vel_pid_pub = rospy.Publisher("cmd_vel/pid", Twist, queue_size=10)

        cmd_vel_pid_msg = Twist()
        self.tf_listener = tf.TransformListener()   # Do I need to transform or is everything in Odom fram already?

        # init pid vars
        # linear

        # angular


        while not rospy.is_shutdown():

            cmd_vel_pid_msg.linear.x = self.pid_linear()
            cmd_vel_pid_msg.angular.z = self.pid_angular()

            cmd_vel_pid_pub.publish(cmd_vel_pid_msg)
            r.sleep()

    def pid_linear(self):
        None

    def pid_angular(self):
        None
