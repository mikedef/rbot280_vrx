#!/usr/bin/env python                                                                                                                      

# Michael DeFilippo, 2022-03-13                                                                                                            
'''                                                                                                                                        
Goal is to smoothly traverse to waypoint on the path provided by the move_base package. 
Using instances of the ros pid package to robot desired speed and desired heading.
'''                                                                                                                                        

import rospy
from math import pi
from std_msgs.msg import Float64, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from rbot280.cfg import DesiredHdgSpdDynamicConfig

class Node(object):
        
    def __init__(self):
        rospy.init_node("desired heading and speed control")
        r = rospy.Rate(5)

        # Publishers
        self.setpoint_speed = rospy.Publisher("speed/setpoint", Float64, queue_size=10)
        self.setpoint_heading = rospy.Publisher("heading/setpoint", Float64, queue_size=10)
        self.speed_state_pub = rospy.Publisher("speed/state", Float64, queue_size=10)
        self.heading_state_pub = rospy.Publisher("heading/state", Float64, queue_size=10)
        self.left_pub = rospy.Publisher("wamv/thrusters/left_thrust_cmd", Float32, queue_size=10)
        self.right_pub = rospy.Publisher("wamv/thrusters/right_thrust_cmd", Float32, queue_size=10)

        # Subscriber
        rospy.Subscriber("/speed/control_effort", Float64, self.speed_control_effort_cb, queue_size=10)
        rospy.Subscriber("/heading/control_effort", Float64, self.heading_control_effort_cb, queue_size=10)
        rospy.Subscriber("/wamv/robot_localization/odometry/filtered", Odometry, self.robot_cb, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb, queue_size=10)

        # Dynamic Configure
        self.srv = Server(DesiredHdgSpdDynamicConfig, self.dynamic_cb)

        self.desired_speed = Float64()
        self.desired_heading = Float64()
        self.speed_plant_state = Float64()
        self.speed_control_effort = Float64()
        self.heading_plant_state = Float64()
        self.heading_control_effort = Float64()
        self.dx = Float64()
        self.dyaw = Float64()
        self.left_cmd = Float32()
        self.right_cmd = Float32()
        self.cmd_vel_x = Float32()
        self.cmd_ang_z = Float32()
        
        self.dx.data = 0.0
        self.dyaw.data = 0.0
        self.speed_control_effort.data = 0.0
        self.heading_control_effort.data = 0.0
        self.cmd_vel_x.data = 0.0
        self.cmd_ang_z.data = 0.0
    
        self.desired_speed.data = rospy.get_param('~desired_speed', 0.0)
        self.desired_heading.data = rospy.get_param('~desired_heading', 0.0)
        rospy.loginfo(self.desired_speed)

        while not rospy.is_shutdown():
            self.hdg_cb()
            #self.spd_cb()

            self.setpoint_heading.publish(self.desired_heading)
            self.setpoint_speed.publish(self.desired_speed)

            r.sleep()

    def hdg_cb(self):
        self.desired_heading = self.desired_heading
        self.heading_state_pub.publish(0.0)

    def spd_cb(self):
        self.desired_speed = self.desired_speed
        #rospy.loginfo("desired_speed: %f"%self.desired_speed.data)
        control = self.speed_control_effort.data
        dx = self.dx.data
        self.speed_plant_state.data = self.speed_control_effort.data + self.dx.data
        if self.speed_plant_state.data > self.desired_speed.data:
            self.speed_plant_state.data = self.desired_speed.data
        self.speed_state_pub.publish(self.speed_plant_state.data)

        torque = 0.0  # dummy torque for scaling 
        # scale to diff drive
        self.left_cmd.data = -1.0 * torque + self.speed_plant_state.data
        self.right_cmd.data = torque + self.speed_plant_state.data
        
        self.left_pub.publish(self.left_cmd)
        self.right_pub.publish(self.right_cmd)

    def speed_control_effort_cb(self, msg):
        self.speed_control_effort.data = msg.data

    def robot_cb(self, msg):
        self.dx.data = msg.twist.twist.linear.x
        self.dyaw.data = msg.twist.twist.angular.z
        
    def heading_control_effort_cb(self, msg):
        self.heading_control_effort.data = msg.data

    def cmd_vel_cb(self, msg):
        self.cmd_vel_x.data = msg.linear.x
        self.cmd_ang_z.data = msg.angular.z
        # If being commanded to drive Do PID loop
        self.spd_cb()
        

    def dynamic_cb(self, config, level):
        rospy.loginfo("Reconfigure Request...")

        self.desired_speed = config['desired_speed']
        self.desired_heading = config['desired_heading']

        return config

if __name__ == '__main__':
    try:                                                                                                                                   
        node = Node()                                                                                                        
        
    except rospy.ROSInterruptException:                                                                                                    
        pass 


    

    
    
