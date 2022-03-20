#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class PlayerDetectionNode(object):
  def __init__(self):
    """Initializes the node"""
    self.node_name = rospy.get_name()

    # Get parameters
    self.rate = rospy.get_param("~rate", 10)
    self.screen_msg = rospy.get_param("~screen_msg")

    # Setup the publisher and subscriber
    self.sub = rospy.Subscriber(self.screen_msg, Image, self.callback)
    self.pub = rospy.Publisher('~test', String, queue_size=1)

  def callback(self, msg):
    hello = "hello world"
    rospy.loginfo(hello)
    self.pub.publish(hello)

  def loginfo(self, msg):
    rospy.loginfo(f"[{self.node_name}] {msg}")

if __name__ == '__main__':
  rospy.init_node('player_detection_node', anonymous=False)
  detection_node = PlayerDetectionNode()
  rospy.spin()