#!/usr/bin/env python
import rospy

import actionlib

import move_base_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseAction(object):
    # create messages that are used to publish feedback/result
    _feedback = move_base_msgs.msg.MoveBaseFeedback()
    _result = move_base_msgs.msg.MoveBaseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.pub = rospy.Publisher('/rtabmap/initialpose', PoseWithCovarianceStamped, queue_size=10)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, sending inital pose to RTABMAP' % (self._action_name))
        
        # Fill pose msg with message from action goal
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose = goal.target_pose.pose
        pose_msg.header.stamp = rospy.get_rostime()
        pose_msg.header.frame_id = 'map'
        self.pub.publish(pose_msg)

        rospy.loginfo(pose_msg)
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('initial_pose_action')
    server = InitialPoseAction(rospy.get_name())
    rospy.spin()
