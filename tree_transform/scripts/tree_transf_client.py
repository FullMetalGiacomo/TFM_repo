#!/usr/bin/env python

import rospy
import rospkg

from tree_transform.srv import tree_transformService, tree_transformResponse
from data_fusion_node.msg import PoseArrayId
from apriltag_ros.msg import AprilTagDetectionArray

def get_tree_transf_client():
    rospy.wait_for_service('tree_transf_srv')
    try:
        rospy.logwarn("here")
        client_handle = rospy.ServiceProxy('tree_transf_srv', tree_transformService)
	# wait topic
        detections=rospy.wait_for_message("/poses_pfc", PoseArrayId)
        tag=rospy.wait_for_message("/tag_detections", AprilTagDetectionArray)

        response1 = client_handle(tag,detections.poses)
        rospy.logwarn(response1)
        return response1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('tree_transform_client')
    get_tree_transf_client()
