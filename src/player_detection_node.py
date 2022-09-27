#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
from Sequoia.detector import SequoiaDetector

class PlayerDetectionNode(object):
  def __init__(self):
    """Initializes the node"""
    # Get parameters
    self.rate = rospy.get_param("~rate", 10)
    self.screen_msg = rospy.get_param("~screen_msg")
    self.visualize = rospy.get_param("~viz", default=False)
    wdir = rospy.get_param("~weights_dir", default=None)

    # Setup
    self.node_name = rospy.get_name()
    self.bridge = CvBridge()
    self.detector = SequoiaDetector(weights_dir=wdir)
    self.latest_img = None
    self.labels = ['t', 'ct']

    # Setup the publisher and subscriber
    self.sub = rospy.Subscriber(self.screen_msg, Image, self.callback)
    self.pubs = {label: rospy.Publisher('~' + label, BoundingBox2DArray, queue_size=1) for label in self.labels}
    self.pub_viz = rospy.Publisher('~bbox_visualized', Image, queue_size=1)


  def callback(self, msg):
    self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    self.publish()

  def publish(self):
    detections = self.detector.detect(self.latest_img)

    for label in self.labels:
      msg = BoundingBox2DArray()
      pub = self.pubs[label]

      for x1, y1, x2, y2 in detections[label]:
        ctr = Pose2D()
        ctr.x = (x2+x1) / 2
        ctr.y = (y2+y1) / 2

        box = BoundingBox2D()
        box.center = ctr
        box.size_x = x2-x1
        box.size_y = y2-y1

        msg.boxes.append(box)

      pub.publish(msg)

    if self.visualize:
      img = self.detector.visualize()
      img_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")

      self.pub_viz.publish(img_msg)

  def loginfo(self, msg):
    rospy.loginfo(f"[{self.node_name}] {msg}")

if __name__ == '__main__':
  rospy.init_node('player_detection_node', anonymous=False)
  detection_node = PlayerDetectionNode()
  rospy.spin()