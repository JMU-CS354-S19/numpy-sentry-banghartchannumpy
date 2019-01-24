#!/usr/bin/env python

""" 
SentryBot lets us know if an intruder walks past.

Author: 
Version:
"""

import rospy

from kobuki_msgs.msg import Sound
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


class SentryNode(object):
    """Monitor a vertical scan through the depth map and create an
    audible signal if the change exceeds a threshold.

    Subscribes:
         /camera/depth_registered/image
       
    Publishes:
        /mobile_base/commands/sound

    """

    def __init__(self):
        """ Set up the Sentry node. """
        rospy.init_node('sentry')
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/depth_registered/image',
                         Image, self.depth_callback, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
        self.sound = Sound()
        self.sound.value = 1
        self.previous_slice = None
        self.threshold = 2
        self.average = 0
        self.alpha = .75
        rospy.spin()

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """
        # Convert the depth message to a numpy array
        
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        current_slice = depth[:,depth_msg.width/2]

        if self.previous_slice != None :
            d = current_slice - self.previous_slice
            d = d[~np.isnan(d)]
            d = np.linalg.norm(d)
            self.average = self.average * self.alpha + d * (1 - self.alpha)
            print d/self.average
            if d/self.average > self.threshold :
                self.sound_pub.publish(self.sound)
        self.previous_slice = current_slice


if __name__ == "__main__":
    SentryNode()
