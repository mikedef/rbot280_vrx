#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage

class Node(object):
    def __init__(self):
        self.pub_img = rospy.Publisher('test_img', Image, queue_size=1)
        self.sub_img = rospy.Subscriber('/wamv/sensors/cameras/front_center_camera/image_raw', Image, self.img_cb)

    def img_cb(self, msg):
        self.pub_img.publish(msg)

if __name__ == '__main__':

    try:
        rospy.init_node('pubsub_img', anonymous=True)
        
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
