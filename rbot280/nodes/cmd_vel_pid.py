#!/usr/bin/env python

# Michael DeFilippo, 2022-03-07
'''
Node that subscribes to /cmd_vel and path to goal to get PID modified linear.x and angular.z 
velocity commands to next waypoint on path. Will republish updated velocity command on 
/cmd_vel/pid 
'''

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from dynamic_reconfigure.server import Server
from rbot280.cfg import CmdVelPIDDynamicConfig


class Cmd_Vel_PID(object):
    velKp = 1.0  # linear proportional constant
    velKi = 0.0  # linear integral constant
    velKd = 0.0  # linear derivative constant
    yawKp = 1.0  # angular proportional constant
    yawKi = 0.0  # angular integral constant
    yawKd = 0.0  # angular derivative constant
    angular_z, linear_x = 0, 0  # value from cmd_vel
    robot_x, robot_y, robot_yaw = 0, 0, 0  # robot pose and position from robot_localization Odometry
    goal_x, goal_y, goal_yaw = 0, 0, 0  # goal pose and position

    # *************** Review thresholds for PID *************************************
    linear_threshold = 5.0
    vel_thresh = 2
    angular_velocity_threshold = 1.5

    def __init__(self):
        # init rospy
        rospy.init_node('cmd_vel_pid', anonymous=True)
        r = rospy.Rate(2)
        # Subscribers
        rospy.Subscriber("cmd_vel", Twist, callback=self.cmd_vel_cb, queue_size=10)
        # Start with global planner but try to transition to local path planner
        rospy.Subscriber("move_base_node/GlobalPlanner/plan", Path, callback=self.goal_cb, queue_size=10)
        rospy.Subscriber("wamv/robot_localization/odometry/filtered", Odometry, callback=self.robot_cb, queue_size=10)
    
        # Publishers                                                                                                         
        self.left_pub = rospy.Publisher('wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)                             
        self.right_pub = rospy.Publisher('wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)                           
        self.vpubdebug_error = rospy.Publisher("vel_pid_debug/error",Float32,queue_size=10)                                 
        self.vpubdebug_setpoint = rospy.Publisher("vel_pid_debug/setpoint",Float32,queue_size=10)
        self.ypubdebug_error = rospy.Publisher("yaw_pid_debug/error",Float32,queue_size=10)                              
        self.ypubdebug_setpoint = rospy.Publisher("yaw_pid_debug/setpoint",Float32,queue_size=10)

        # Dynamic Configure                                                                                              
        self.srv = Server(CmdVelPIDDynamicConfig, self.dynamic_cb) 
        
        # ROS Params                                                                                                         
        self.velKp = rospy.get_param('~velKp', 0.0)                                                                         
        self.velKi = rospy.get_param('~velKi', 0.0)                                                                          
        self.velKd = rospy.get_param('~velKd', 0.0)
        self.yawKp = rospy.get_param('~yawKp', 0.0)
        self.yawKi = rospy.get_param('~yawKi', 0.0)
        self.yawKd = rospy.get_param('~yawKd', 0.0)
        
        # Msg out for left and right thrusters                                                                               
        self.left_cmd = Float32()                                                                                            
        self.right_cmd = Float32()                                                                                                
        # init pid vars
        # linear
        self.error_linear = 0.0
        # setpoint desired_velocity
        self.desired_velocity = 1.0
        self.robot_dyaw = 0.0
        self.robot_dx = 0.0
        # angular
        self.error_yaw = 0.0
        self.desired_yaw = 0.0  # Setpoint

        while not rospy.is_shutdown():

            #cmd_vel_pid_msg.linear.x = self.pid_linear()
            #cmd_vel_pid_msg.angular.z = self.pid_angular()
            #cmd_vel_pid_pub.publish(cmd_vel_pid_msg)
            

            
            r.sleep()

    def pid(self):
        """ Linear PID """
        # self.error_linear
        # self.desired_velocity_x
        self.error_linear = math.sqrt((self.goal_y - self.robot_y)**2 + (self.goal_x - self.robot_x)**2)
        p = self.error_linear * self.velKp
        rospy.loginfo("error * Kp: %f * %f = %f"%(self.error_linear, self.velKp, p))
        rospy.loginfo("p term: %f"%p)

        if self.error_linear < self.linear_threshold:
            pid_linear_x = p # + i + d
        else:
            pid_linear_x = 0.0
        
        thrust = self.linear_x + pid_linear_x

        if thrust > self.vel_thresh:
            thrust = self.vel_thresh
        elif thrust < -self.vel_thresh:
            thrust = -self.vel_thresh
        rospy.loginfo("thrust: %f"%thrust)

        """ Angular PID """
        angle_error = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x) - self.robot_yaw
        # map to -pi to pi
        self.error_angular = math.atan2(math.sin(angle_error), math.cos(angle_error))
        yaw_p = self.yawKp * self.error_angular
        rospy.loginfo("robot dyaw: %f"%self.robot_dyaw)
        rospy.loginfo("error * Kp: %f * %f = %f"%(self.error_angular, self.yawKp, p))                                    
        rospy.loginfo("yaw_p term: %f"%yaw_p)
        if self.angular_z > 0:
            pid_angular_z = yaw_p # + yaw_i - yaw_d
        else:
            pid_angular_z = yaw_p # + yaw_i + yaw_d

        if math.sqrt((self.goal_y - self.robot_y)**2 + (self.goal_x - self.robot_x)**2) > 1:
            torque = self.angular_z + pid_angular_z
        else:
            torque = self.angular_z
        

        # scale to diff drive                                                                                        
        self.left_cmd.data = -1.0 * torque + thrust                                                                          
        self.right_cmd.data = torque + thrust                                                                                
        
        # publish                                                                                                            
        self.left_pub.publish(self.left_cmd)                                                                                 
        self.right_pub.publish(self.right_cmd)

        # Debug                                                                                                      
        self.vpubdebug_error.publish(self.error_linear)                                                                      
        self.vpubdebug_setpoint.publish(1)
        self.ypubdebug_error.publish(self.error_angular)
        self.ypubdebug_setpoint.publish(1)

    def pid_angular(self):
        """ Angular PID """
        # self.desired_yaw
        # self.error_yaw
        return 0.0

    def dynamic_cb(self, config, level):
        rospy.loginfo("Reconfigure Request...")

        self.velKp = config['velKp']
        self.velKi = config['velKi']
        self.velKd = config['velKd']
        self.yawKp = config['yawKp']
        self.yawKi = config['yawKi']
        self.yawKd = config['yawKd']

        #self.desired_velocity_x = config['desired_velocity_x']

        return config

    def cmd_vel_cb(self, msg):
        """ cmd_vel_cb from the subscriber to get pose data """
        self.linear_x = msg.linear.x  # Forward velocity (m/s)
        self.angular_z = msg.angular.z  # Rotational velocity (rad/s)
        #rospy.loginfo("x: %f, y: %f", self.linear_x, self.angular_z)

    def goal_cb(self, msg):
        #rospy.loginfo(msg.poses[0].pose.orientation.z)  # Figure out what is being published here and use the first as the first pose as the goal
        self.goal_x = msg.poses[0].pose.position.x
        self.goal_y = msg.poses[0].pose.position.y
        q = msg.poses[0].pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

        self.error_linear = 0.0  # reset linear error
        self.desired_yaw = 0.0  # reset desired yaw
        self.error_yaw = 0.0  # reset yaw error
        self.pid()

    def robot_cb(self, msg):
        # rospy.loginfo(msg)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_dyaw = msg.twist.twist.angular.z
        self.robot_dx = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        

if __name__ == '__main__':
    try:
        cmd_vel_pid = Cmd_Vel_PID()
        
    except rospy.ROSInterruptException:
        pass
