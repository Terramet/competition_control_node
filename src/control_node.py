#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import random

class GenericControlNode:
    def __init__(self):
        rospy.init_node('generic_control_node')

        # Publisher for ARI robot state
        self.state_pub = rospy.Publisher('/ari_state', String, queue_size=10)

        # Initialize current state
        self.current_state = None

        # Set up a 30-second timer to update the state
        rospy.Timer(rospy.Duration(30.0), self.state_timer_cb)

        rospy.loginfo("GenericControlNode initialized, publishing ARI states every 30 seconds.")
        rospy.spin()

    def state_timer_cb(self, event):
        # Randomly choose between Search and Assess
        self.current_state = random.choice(['Search', 'Assess'])
        rospy.loginfo(f"[ControlNode] Switching to state: {self.current_state}")
        self.state_pub.publish(String(self.current_state))

if __name__ == '__main__':
    try:
        GenericControlNode()
    except rospy.ROSInterruptException:
        pass

