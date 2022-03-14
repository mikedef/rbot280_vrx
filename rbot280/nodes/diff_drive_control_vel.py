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

        # Setup Velocity PID
        self.vpid = Pid(0.0, 0.0, 0.0)
        #self.vpid.set_setpoint(1.0)
        self.vpid.set_setpoint(0.0)
        self.vpid.set_maxIout(1.0)
        self.vpid.set_derivfeedback(True) # D term in feedback loop
        fc = 20;  # cutoff freq
        wc = fc*(2.0*pi)
        self.vpid.set_derivfilter(1,wc)

        # init
        self.lasttime = None
        self.dx = None

    def twist_cb(self, msg):
        rospy.loginfo("velx: %.7f"%float(msg.linear.x))
        vel = msg.linear.x
        self.vpid.set_setpoint(msg.linear.x)
        #self.vpid.set_setpoint(1.0)

        now = rospy.get_time()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now - self.lasttime
        self.lasttime = now
        if dt < 1.0e-6:
            rospy.logwarn("USV Control dt too small <%f>"%dt)
            return
        
        torque = 0

        rospy.loginfo("dx: %f"%self.dx)
        vout = self.vpid.execute(dt, self.dx)
        thrust = vout[0]
        
        
        # scale to diff drive
        self.left_cmd.data = -1.0 * torque + thrust
        self.right_cmd.data = torque + thrust

        # publish
        self.left_pub.publish(self.left_cmd)
        self.right_pub.publish(self.right_cmd)

        # Debug
        self.vpubdebug_error.publish(vout[4])
        self.vpubdebug_setpoint.publish(vout[5])
        

    def odom_cb(self, msg):
                
        
        #if self.lastx is None:
        #    #self.lastx = msg.pose.position.x
        #    return
        #x = msg

        
        # velocity control
        dx = msg.twist.twist.linear.x
        self.dx = dx
        #rospy.loginfo("dx: %.7f"%dx)
        if dx < 0.01:
            dx = 0.0
        #rospy.loginfo("dx: %.7f"%dx)

        
        
        
    def dynamic_cb(self, config, level):
        rospy.loginfo("PID Reconfigure request...")
        # Use method to zero the integrator
        tol = 1e-6
        
        self.vpid.Kp = config['velKp']
        rospy.loginfo("velKp %.3f"%self.vpid.Kp)
        Ki = config['velKi']
        if abs(abs(Ki) - abs(self.vpid.Ki)) > tol:
            rospy.loginfo("Setting vel Ki to %.3f"%Ki)
            self.vpid.set_Ki(Ki)
        self.vpid.Kd = config['velKd']
        return config

if __name__ == '__main__':

    rospy.init_node('diff_drive_twist_control', anonymous=True)

    # ROS Params
    velKp = rospy.get_param('~velKp', 0.0)
    velKi = rospy.get_param('~velKi', 0.0)
    velKd = rospy.get_param('~velKd', 0.0)

    # Init Cmd_Vel_PID object - creats a PID object
    node = Node()

    # Setup gains from params
    node.vpid.Kp = velKp
    node.vpid.Ki = velKi
    node.vpid.Kd = velKd
    
    # Msg out for left and right thrusters
    node.left_cmd = Float32()
    node.right_cmd = Float32()

    # Publishers
    node.left_pub = rospy.Publisher('left_thrust_cmd', Float32, queue_size=1)
    node.right_pub = rospy.Publisher('right_thrust_cmd', Float32, queue_size=1)

    node.vpubdebug_error = rospy.Publisher("vel_pid_debug/error",Float32,queue_size=10)
    node.vpubdebug_setpoint = rospy.Publisher("vel_pid_debug/setpoint",Float32,queue_size=10)

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
