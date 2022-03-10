#!/usr/bin/env python                                                                                   
                                                                                                        
# Michael DeFilippo, 2022-03-07                                                                         
'''                                                                                                     
PID control of the /cmd_vel Twist message from move_base. Goal is to smoothly traverse
to waypoint based on these messages. Using instances of the pypid class to control yaw and velocity
'''                                                                                                     
                                                                                                        
import rospy                                                                                            
import math                                                                                             
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from rbot280.cfg import CmdVelPIDDynamicConfig                                     
                                                                                            
class Cmd_Vel_PID(object):
    def __init__(self):


        # Setup Yaw PID

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
    pid = Cmd_Vel_PID()

    # Setup gains from params
    pid.vpid.Kp = velKp
    pid.vpid.
