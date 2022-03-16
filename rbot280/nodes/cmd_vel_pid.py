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
from move_base_msgs.msg import MoveBaseActionGoal
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
        r = rospy.Rate(10)
        # Subscribers
        rospy.Subscriber("cmd_vel", Twist, callback=self.cmd_vel_cb, queue_size=10)
        # Start with global planner but try to transition to local path planner
        #rospy.Subscriber("move_base_node/GlobalPlanner/plan", Path, callback=self.goal_cb, queue_size=10)
        rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callback=self.test_goal_cb, queue_size=10)
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
        self.thrust = 0.0
        self.integrator_max_linear = 100
        self.integrator_min_linear = -100
        self.derivator_linear = 0.0
        self.integrator_linear = 0.0
        # angular
        self.integrator_max_yaw = 100
        self.integrator_min_yaw = -100
        self.derivator_yaw  = 0.0
        self.integrator_yaw = 0.0
        self.error_yaw = 0.0
        self.desired_yaw = 0.0  # Setpoint
        self.torque = 0.0

        while not rospy.is_shutdown():

            #cmd_vel_pid_msg.linear.x = self.pid_linear()
            #cmd_vel_pid_msg.angular.z = self.pid_angular()
            #cmd_vel_pid_pub.publish(cmd_vel_pid_msg)

            self.pid_linear()
            self.pid_angular
            
            # scale to diff drive                                                                                        
            self.left_cmd.data = -1.0 * self.torque + self.thrust                                                                          
            self.right_cmd.data = self.torque + self.thrust                                                                                
            
            # publish                                                                                                            
            self.left_pub.publish(self.left_cmd)                                                                                 
            self.right_pub.publish(self.right_cmd)
            
            r.sleep()

    def pid(self):
        """ Linear PID """
        # self.error_linear
        # self.desired_velocity_x
        rospy.loginfo("goal: %f,%f"%(self.goal_x, self.goal_y))
        rospy.loginfo("robot: %f,%f"%(self.robot_x, self.robot_y))
        self.error_linear = math.sqrt((self.goal_y - self.robot_y)**2 + (self.goal_x - self.robot_x)**2)
        p = self.error_linear * self.velKp
        rospy.loginfo("error * Kp: %f * %f = %f"%(self.error_linear, self.velKp, p))
        #rospy.loginfo("p term: %f"%p)

        if self.error_linear < self.linear_threshold:
            pid_linear_x = p # + i + d
        else:
            pid_linear_x = 0.0
        
        self.thrust = self.linear_x + pid_linear_x

        if thrust > self.vel_thresh:
            self.thrust = self.vel_thresh
        elif thrust < -self.vel_thresh:
            self.thrust = -self.vel_thresh
        #rospy.loginfo("thrust: %f"%thrust)

        """ Angular PID """
        angle_error = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x) - self.robot_yaw
        # map to -pi to pi
        self.error_angular = math.atan2(math.sin(angle_error), math.cos(angle_error))
        yaw_p = self.yawKp * self.error_angular
        if self.angular_z > 0:
            pid_angular_z = yaw_p # + yaw_i - yaw_d
        else:
            pid_angular_z = yaw_p # + yaw_i + yaw_d

        if math.sqrt((self.goal_y - self.robot_y)**2 + (self.goal_x - self.robot_x)**2) > 1:
            self.torque = self.angular_z + pid_angular_z
        else:
            self.torque = self.angular_z
        rospy.loginfo("torque: %f"%torque)

        # scale to diff drive                                                                                        
        self.left_cmd.data = -1.0 * torque + thrust                                                                          
        self.right_cmd.data = torque + thrust                                                                                
        
        # publish                                                                                                            
        self.left_pub.publish(self.left_cmd)                                                                                 
        self.right_pub.publish(self.right_cmd)

        # Debug                                                                                                      
        
        self.ypubdebug_error.publish(self.error_angular)
        self.ypubdebug_setpoint.publish(1)

        return (self.thrust)
    
    def pid_linear(self):
        self.error_linear = math.sqrt((self.goal_y - self.robot_y)**2 + (self.goal_x - self.robot_x)**2)
        p = self.error_linear * self.velKp
        rospy.loginfo("error * Kp: %f * %f = %f"%(self.error_linear, self.velKp, p))
        #rospy.loginfo("p term: %f"%p)

        if self.error_linear < self.linear_threshold:
            pid_linear_x = p # + i + d
        else:
            pid_linear_x = 0.0
        
        self.thrust = self.linear_x + pid_linear_x

        if self.thrust > self.vel_thresh:
            self.thrust = self.vel_thresh
        elif self.thrust < -self.vel_thresh:
            self.thrust = -self.vel_thresh

        self.vpubdebug_error.publish(self.error_linear)                                                                      
        self.vpubdebug_setpoint.publish(self.linear_threshold)

        return (self.thrust)

    def pid_angular(self):
        """ Angular PID """
        
        angle_error = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x) - self.robot_yaw
        # map to -pi to pi
        self.error_angular = math.atan2(math.sin(angle_error), math.cos(angle_error))
        self.yaw_p = self.yawKp * self.error_angular
        derivative_error = self.error_angular - self.derivator_yaw
        self.yaw_d = self.yawKd * math.fabs(math.atan2(math.sin(derivative_error), math.cos(derivative_error)))
        self.derivator_yaw = self.error_angular
        self.integrator_yaw = self.integrator_yaw + self.error_angular

        if self.integrator_yaw > self.integrator_max_yaw:
            self.integrator_yaw = self.integrator_max_yaw
        elif self.integrator_yaw < self.integrator_min_yaw:
            self.integrator_yaw = self.integrator_min_yaw

        self.yaw_i = self.integrator_yaw * self.yawKi
        
        if self.angular_z > 0:
            pid_angular_z = self.yaw_p + self.yaw_i - self.yaw_d
        else:
            pid_angular_z = self.yaw_p + self.yaw_i + self.yaw_d

        if math.sqrt((self.goal_y - self.robot_y)**2 + (self.goal_x - self.robot_x)**2) > 1:
            self.torque = self.angular_z + pid_angular_z
        else:
            self.torque = self.angular_z
        rospy.loginfo("torque: %f"%torque)
        return (self.torque)

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
        self.derivator_linear = 0.0
        self.integrator_linear = 0.0
        self.derivator_yaw  = 0.0
        self.integrator_yaw = 0.0
        
        self.pid()

    def robot_cb(self, msg):
        # rospy.loginfo(msg)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_dyaw = msg.twist.twist.angular.z
        self.robot_dx = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

    def test_goal_cb(self, msg):
        #rospy.loginfo(msg.poses[0].pose.orientation.z)  # Figure out what is being published here and use the first as the first pose as the goal
        self.goal_x = msg.goal.target_pose.pose.position.x
        self.goal_y = msg.goal.target_pose.pose.position.y
        q = msg.goal.target_pose.pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

        self.error_linear = 0.0  # reset linear error
        self.desired_yaw = 0.0  # reset desired yaw
        self.error_yaw = 0.0  # reset yaw error
        #self.pid()

if __name__ == '__main__':
    try:
        cmd_vel_pid = Cmd_Vel_PID()
        
    except rospy.ROSInterruptException:
        pass
