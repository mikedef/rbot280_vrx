#!/usr/bin/env python                                                                                   
                                                                                                        
# Michael DeFilippo, 2022-03-07                                                                         
'''                                                                                                     
PID control of the /cmd_vel Twist message from move_base. Goal is to smoothly traverse
to waypoint based on these messages. Using instances of the pypid class to control yaw and velocity
Addapted from kingfisher_control
'''                                                                                                     
                                                                                                        
import rospy                                                                                            
from math import pi
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from rbot280.cfg import CmdVelPIDDynamicConfig

from usv_pid.pypid import Pid
                                                                                            
class Node(object):
    def __init__(self):

        # Setup Yaw PID
        self.ypid = Pid(0.0, 0.0, 0.0)
        self.ypid.set_setpoint(0.0)
        #self.ypid.set_inputisangle(True,pi)
        self.ypid.set_derivfeedback(True)
        fc = 20  # cutoff frequency
        wc = fc*(2.0*pi)  # cutoff freq in rad/s
        self.ypid.set_derivfilter(1,wc)
        self.ypid.set_maxIout(1.0)
        
        # init
        self.lasttime = None


    def twist_cb(self, msg):
        rospy.loginfo("vel: %f, ang: %f"%(msg.linear.x, msg.angular.z))
        self.ypid.set_setpoint(msg.angular.z)
        
        

    def odom_cb(self, msg):
        # yaw control
        dyaw = msg.twist.twist.angular.z  # measured rate (process variable)

        now = rospy.get_time()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now - self.lasttime
        self.lasttime = now
        if dt < 1.0e-6:
            rospy.logwarn("USV Control dt too small <%f>"%dt)
            return
        
        yout = self.ypid.execute(dt, dyaw)
        torque = yout[0]

        thrust = 0

        # scale to diff drive
        self.left_cmd.data = -1.0 * torque + thrust
        self.right_cmd.data = torque + thrust

        # publish
        self.left_pub.publish(self.left_cmd)
        self.right_pub.publish(self.right_cmd)

        # Debug
        self.ypubdebug_error.publish(yout[4])
        self.ypubdebug_setpoint.publish(yout[5])
                
        
    def dynamic_cb(self, config, level):
        rospy.loginfo("PID Reconfigure request...")
        self.ypid.Kp = config['yawKp']
        
        # Use method to zero the integrator
        Ki = config['yawKi']
        tol = 1e-6
        if abs(abs(Ki) - abs(self.ypid.Ki)) > tol:
            rospy.loginfo("Setting yaw Ki to %.3f"%Ki)
            self.ypid.set_Ki(Ki)
        self.ypid.Kd = config['yawKd']

        return config

if __name__ == '__main__':

    rospy.init_node('diff_drive_twist_control', anonymous=True)

    # ROS Params
    velKp = rospy.get_param('~velKp', 0.0)
    velKi = rospy.get_param('~velKi', 0.0)
    velKd = rospy.get_param('~velKd', 0.0)

    yawKp = rospy.get_param('~yawKp', 0.0)
    yawKi = rospy.get_param('~yawKi', 0.0)
    yawKd = rospy.get_param('~yawKd', 0.0)

    # Init Cmd_Vel_PID object - creats a PID object
    node = Node()

    # Setup gains from params
    node.ypid.Kp = yawKp
    node.ypid.Ki = yawKi
    node.ypid.Kd = yawKd

    # Msg out for left and right thrusters
    node.left_cmd = Float32()
    node.right_cmd = Float32()

    # Publishers
    node.left_pub = rospy.Publisher('left_thrust_cmd', Float32, queue_size=1)
    node.right_pub = rospy.Publisher('right_thrust_cmd', Float32, queue_size=1)

    node.ypubdebug_error = rospy.Publisher("yaw_pid_debug/error",Float32,queue_size=10)
    node.ypubdebug_setpoint = rospy.Publisher("yaw_pid_debug/setpoint",Float32,queue_size=10)
    
    # Subscribers
    s1 = rospy.Subscriber("cmd_vel", Twist, node.twist_cb )                                   
    s2 = rospy.Subscriber("wamv/robot_localization/odometry/filtered", Odometry, node.odom_cb) 

    # print info
    rospy.loginfo("Publishing to %s and %s"%(node.left_pub.name, node.right_pub.name))
    rospy.loginfo("Subscribing to %s and %s"%(s1.name, s2.name))

    # Dynamic Configure
    srv = Server(CmdVelPIDDynamicConfig, node.dynamic_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
