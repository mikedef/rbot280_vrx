#!/usr/bin/env python3
"""

"""

from threading import Lock
import rospy
import numpy as np
import cv2
import darknet
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CompressedImage
from rbot_msgs.msg import Classification, ClassificationArray
from rbot_msgs import utils

class darknet_node(object):
    """
        Node to perform object detection

        Subscriptions: 
            - ~[0,...,~n_inputs]/input:   [sensor_msgs/CompressedImage] rgb camera image

        Publications:  
            - ~[0,...,~n_inputs]/output:  [asv_perception_common/ClassificationArray]
                        Array of detections.  Resulting ROIs scaled to input image size

            - ~[0,...,~n_inputs]/image:   [sensor_msgs/Image] annotated image (for visualization/debugging)
        
        Parameters:
            - ~n_inputs:    [int, default=1]  number of ~input[0...n] subscriptions and corresponding publications
            - ~classes:     [array[string], default=None]  list of classifier classes to emit.  must be lower case.  If empty, no restriction
            - ~darknet_config_file: [string]  path to darknet config file
            - ~darknet_weights_file: [string]  path to darknet weights file
            - ~darknet_meta_file: [string]  path to darknet meta file
            - ~darknet_thresh: [float, default=0.5]  darknet threshold
            - ~darknet_hier_thresh: [float, default=0.5]  darknet hier_thres
            - ~darknet_nms: [float, default=0.45]  darknet nms
    """

    def __init__(self):

        self.node_name = rospy.get_name()
        self.lock = Lock()

        self.n_inputs = rospy.get_param("~n_inputs", 1 )
        self.classes = rospy.get_param( "~classes", [] )

        # init darknet
        configPath = rospy.get_param("~darknet_config_file")
        weightPath = rospy.get_param("~darknet_weights_file")
        dataPath = rospy.get_param("~darknet_data_file")
        

        # Cv bridge
        self.bridge = CvBridge()
        
        #darknet params; using defaults from darknet.py
        self.darknet_thresh = rospy.get_param("~darknet_thresh", 0.5 )
        self.darknet_hier_thresh = rospy.get_param("~darknet_hier_thresh", 0.5 )
        self.darknet_nms = rospy.get_param("~darknet_nms", 0.45 )

        self.network, self.class_names, self.colors = darknet.load_network(
            configPath,
            dataPath,
            weightPath,
            0
        )
              
        # publishers array of classification array
        self.pubs = [ rospy.Publisher( '~%d/output' % i, ClassificationArray, queue_size=1 ) for i in range(self.n_inputs) ]

        # image publishers for visualization/debugging
        self.pubs_img = [ rospy.Publisher( '~%d/image' % i, Image, queue_size=1 ) for i in range(self.n_inputs) ]

        # subscriptions array
        #self.subs = [ rospy.Subscriber( '~%d/input' % i, CompressedImage, queue_size=1, callback=self.cb_sub, callback_args=i, buff_size=2**24 ) for i in range(self.n_inputs) ]
        self.subs = [ rospy.Subscriber( '~%d/input' % i, Image, queue_size=1, callback=self.cb_sub, callback_args=i, buff_size=2**24 ) for i in range(self.n_inputs) ]
        
    def cb_sub( self, msg, idx ):
        """ perform callback for image message at input index """
        
        # get associated publishers for this input index
        pub = self.pubs[idx]
        pub_img = self.pubs_img[idx]
        
        # no subscribers, no work
        #if pub.get_num_connections() < 1 and pub_img.get_num_connections() < 1:
        #    rospy.loginfo("No publishers")
        #    return

        rospy.logdebug( 'Processing img with timestamp secs=%d, nsecs=%d', msg.header.stamp.secs, msg.header.stamp.nsecs )
        
        dets, img = self.detect( msg ) # perform detection

        #pub.publish( dets )
        if pub.get_num_connections() > 0:  # publish detections if something is connected and looking for it?
            pub.publish( dets )
        
        if pub_img.get_num_connections() > 0:  # publish annotated image
            annotated = self.annotate( img, dets )
            pub_img.publish( annotated )
            #pub_img.publish(self.bridge.cv2_to_imgmsg(img, 'rgb8'))

        
    def detect(self, image_msg):
        """ perform object detection in image message, return ClassificationArray, OpenCV Image """
        
        # darknet requires rgb image in proper shape.  we need to resize, and then convert resulting bounding boxes to proper shape
        # need distortion-free center crop; detector likely requires square image while input is likely widescreen
        image_cv = self.bridge.imgmsg_to_cv2(image_msg, 'rgb8')
        image_rgb = cv2.cvtColor(image_cv, cv2.COLOR_BGR2RGB )
        width = darknet.network_width(self.network)
        height = darknet.network_height(self.network)
        orig_shape = image_cv.shape
        image_resized, offsets, scale = utils.resize( image_rgb, ( width, height ) ) # do crop/resize
        scale_up = 1./ float(scale) #invert scale to convert from resized --> orig
        offsets = np.abs(offsets // 2) # divide offsets by 2 for center crop.  make positive

        # convert to darknet img format
        darknet_image = darknet.make_image(width, height, 3)
        darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
        # returns [(nameTag, dets[j].prob[i], (b.x, b.y, b.w, b.h))]:
        detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=self.darknet_thresh,
                                          hier_thresh=self.darknet_hier_thresh, nms=self.darknet_nms)
        darknet.free_image(darknet_image)
        image = darknet.draw_boxes(detections, image_resized, self.colors)
               
        msg = ClassificationArray()
        msg.header = image_msg.header #match timestamps
        msg.image_width = orig_shape[1]
        msg.image_height = orig_shape[0]
        
        for det in detections:
            cls = Classification()
            cls.label = det[0]
            cls.probability = float( det[1] )  # Change to float
            roi = det[2]
            #rospy.loginfo("label: %s, prob: %s, roi: %s"%(det[0], det[1], det[2]))
            # darknet roi:  ( x, y, w, h ), where x and y are the centers of the detection
            #  RegionOfInterest x & y are left- and top-most coords
            cls.roi.width = roi[2]
            cls.roi.height = roi[3]
            cls.roi.x_offset = roi[0] - ( cls.roi.width / 2 )   # convert to left-most x
            cls.roi.y_offset = roi[1] - ( cls.roi.height / 2. )  # convert to top-most y
            
            # scale roi to orig img size
            cls.roi.x_offset *= scale_up
            cls.roi.width *= scale_up
            cls.roi.y_offset *= scale_up
            cls.roi.height *= scale_up
            
            # append crop offset, convert to uint32   ***** np does not work here in py3 ***
            #cls.roi.x_offset = np.uint32( cls.roi.x_offset + offsets[1]  )
            #cls.roi.y_offset = np.uint32( cls.roi.y_offset + offsets[0] )
            #cls.roi.width = np.uint32( cls.roi.width )
            #cls.roi.height = np.uint32( cls.roi.height )
            cls.roi.x_offset = int( cls.roi.x_offset + offsets[1]  )
            cls.roi.y_offset = int( cls.roi.y_offset + offsets[0] )
            cls.roi.width = int( cls.roi.width )
            cls.roi.height = int( cls.roi.height )

            msg.classifications.append(cls)
        
        return msg, image_cv

    def annotate( self, img, clsMsg ):
        """
        Create the annotated image for visualization/debugging
        """

        # upscale/pad img back to orig resolution
        #  bounding boxes are already correct for this resolution
        img = utils.resize( img, ( clsMsg.image_height, clsMsg.image_width ), 'pad' )[0]

        # do color conversion rgb --> bgr (optional)
        #img = cv2.cvtColor( img, cv2.COLOR_RGB2BGR )

        for cls in clsMsg.classifications:
            (w, h) = (cls.roi.width, cls.roi.height)
            (x, y) = ( cls.roi.x_offset, cls.roi.y_offset )
            
            
            # draw a bounding box rectangle and label on the image
            color = (255,0,0)
            cv2.rectangle( img, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format( cls.label, float(cls.probability) )
            cv2.putText( img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2 )

        # create ros msg from img
        msg = utils.convert_cv2_to_ros_msg( img, 'rgb8' )
        msg.header = clsMsg.header # match timestamp
        return msg

if __name__ == "__main__":

    try:
        rospy.init_node(darknet_node.__name__)
        n = darknet_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
