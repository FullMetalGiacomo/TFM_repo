#!/usr/bin/env python

import rospy
import rospkg

from tree_transform.srv import tree_transform as TransformResponse
from geometry_msgs.msg import PoseArray

def get_tree_transf_client():
    rospy.wait_for_service('tree_transf_srv')
    try:
        client_handle = rospy.ServiceProxy('tree_transf_srv', TransformResponse)
        response1 = client_handle()
        return response1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    get_tree_transf_client()
