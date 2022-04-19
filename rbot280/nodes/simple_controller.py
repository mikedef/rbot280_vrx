#!/usr/bin/env python3

# Michael DeFilippo, 2022-04-15
'''
Simple controller to move the wamv to a goal point
'''

import rospy
import math
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from rbot280.cfg import CmdVelPIDDynamicConfig

class Node(object):
    def __init__(self):
        self.goal = Point()
        self.goal_theta = 0.0
        self.angle_to_wpt = 0.0
        self.robot = Point()
        self.torque = 0.0
        self.thrust = 0.0

        self.derivator_yaw = 0.0
        self.integrator_yaw = 0.0
        self.max_integrator_yaw = 10.0
        self.min_integrator_yaw = -10.0

        
    def path_cb(self, msg):
        # Get last point in path list and set as current waypoint goal
        self.goal.x = msg.poses[-1].pose.position.x
        self.goal.y = msg.poses[-1].pose.position.y
        rospy.loginfo("Goal Point: X:%f, Y:%f"%(self.goal.x, self.goal.y))
        rospy.loginfo("Current Point: X:%f, Y:%f"%(self.robot.x, self.robot.y))

        # Drive to goal wpt
        self.control()
        
    def odom_cb(self, msg):
        # Get robot xy
        self.robot.x = msg.pose.pose.position.x
        self.robot.y = msg.pose.pose.position.y
        # Get robot theta
        q = msg.pose.pose.orientation
        (roll, pitch, self.robot.z) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        #rospy.loginfo("robot x,y,theta: %f, %f, %f"%(self.robot.x, self.robot.y, self.robot.z))

        

    def control(self):
        # Find angle difference (if < 0.1 angular.z thrust)
        x = self.goal.x - self.robot.x
        y = self.goal.y - self.robot.y
        theta = math.atan2(y,x)

        # Angular PID
        error_angle = theta - self.robot.z
        # map to -pi to pi
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
        yawP = self.yawKp * error_angle
        rospy.loginfo("Error Angular: %f, yawP: %f"%(error_angle, yawP))
        derivative_error = error_angle - self.derivator_yaw
        yawD = self.yawKd * math.fabs(math.atan2(math.sin(derivative_error),
                                                 math.cos(derivative_error)))
        self.derivator_yaw = error_angle

        self.integrator_yaw = self.integrator_yaw + error_angle
        if self.integrator_yaw > self.max_integrator_yaw:
            self.integrator_yaw = self.max_integrator_yaw
        elif self.integrator_yaw < self.min_integrator_yaw:
            self.integrator_yaw = self.min_integrator_yaw
        yawI = self.yawKi * self.integrator_yaw
                
        # Find distance difference (if less than 2.5m linear.x thrust)
        error_linear = math.sqrt((self.goal.y - self.robot.y)**2 + (self.goal.x - self.robot.x)**2)

        if abs(error_angle) > 0.25:
            self.torque = yawP + yawD  + yawI  # yawP + yawI + yawD
        else:
            self.torque = 0.0

        # Linear PID
        #lin_kp = 1.0
        self.velKp
        velP = self.velKp * error_linear
        thrust = velP # + velI + velD
        #max_speed = 0.4
        if thrust > self.maxSpeed:
            thrust = self.maxSpeed

        rospy.loginfo("linear error: %f"%error_linear)
            
        if error_linear > 3.5:
            self.thrust = thrust
        else:
            self.thrust = 0.0
            self.torque = 0.0

        if error_linear < 3.5:
            rospy.loginfo("At goal WPT")
            # reset PID params
            self.derivator_yaw = 0.0
            self.integrator_yaw = 0.0
            error_angular = 0.0

        # scale to diff drive 
        self.left_cmd.data = -1.0 * self.torque + self.thrust
        self.right_cmd.data = self.torque + self.thrust
        rospy.loginfo("Thrust Left: %f, Thrust Right: %f"%(self.left_cmd.data, self.right_cmd.data))
        
        # publish control to diff drive
        self.left_pub.publish(self.left_cmd)
        self.right_pub.publish(self.right_cmd)

        # debug
        self.ypubdebug_error.publish(error_angle)
        self.ypubdebug_setpoint.publish(0.0)
        self.vpubdebug_error.publish(error_linear)
        self.vpubdebug_setpoint.publish(0.0)

    def dynamic_cb(self, config, level):
        rospy.loginfo("Reconfigure request....")
        self.velKp = config['velKp']
        Ki = config['velKi']
        # zero integrator
        tol = 1e-6
        if abs(abs(Ki) - abs(self.velKi)) > tol:
            rospy.loginfo('setting vel Ki to %.3f'%Ki)
            self.velKi = Ki
        self.velKd = config['velKd']

        self.yawKi = config['yawKi']
        Ki = config['yawKi']
        if abs(abs(Ki) - abs(self.yawKi)) > tol:
            rospy.loginfo('setting yaw Ki to %.3f'%Ki)
            self.yawKi = Ki
        self.velKd = config['velKd']

        self.maxSpeed = config['maxSpeed']

        return config


if __name__ == '__main__':
    rospy.init_node('simple_controller', anonymous=True)

    # Init class
    node = Node()

    velKp = rospy.get_param('~velKp', 0.0)
    velKi = rospy.get_param('~velKi', 0.0)
    velKd = rospy.get_param('~velKd', 0.0)
    yawKp = rospy.get_param('~yawKp', 0.0)
    yawKi = rospy.get_param('~yawKi', 0.0)
    yawKd = rospy.get_param('~yawKd', 0.0)
    maxSpeed = rospy.get_param('~maxSpeed', 0.0)

    # ingest params
    node.velKp = velKp
    node.velKi = velKi
    node.velKd = velKd
    node.yawKp = yawKp
    node.yawKi = yawKi
    node.yawKd = yawKd
    node.maxSpeed = maxSpeed

    # Msg out for left and right thrusters
    node.left_cmd = Float32()
    node.right_cmd = Float32()

    # Publishers
    node.left_pub = rospy.Publisher('left_thrust_cmd', Float32, queue_size=1)
    node.right_pub = rospy.Publisher('right_thrust_cmd', Float32, queue_size=1)

    # Debug
    node.ypubdebug_error = rospy.Publisher('yaw_pid_debug/error', Float32, queue_size=10)
    node.ypubdebug_setpoint = rospy.Publisher('yaw_pid_debug/setpoint', Float32, queue_size=10)
    node.vpubdebug_error = rospy.Publisher('vel_pid_debug/error', Float32, queue_size=10)
    node.vpubdebug_setpoint = rospy.Publisher('vel_pid_debug/setpoint', Float32, queue_size=10)

    # Subscribers
    s1 = rospy.Subscriber("wamv/robot_localization/odometry/filtered", Odometry, node.odom_cb)
    s2 = rospy.Subscriber("move_base_node/GlobalPlanner/plan", Path, node.path_cb)

    # Print sub/pub info for node
    rospy.loginfo("Publishing to %s and %s"%(node.left_pub.name, node.right_pub.name))
    rospy.loginfo("Subscribing to %s and %s"%(s1.name, s2.name))
    
    # Dynamic Configure
    srv = Server(CmdVelPIDDynamicConfig, node.dynamic_cb)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
