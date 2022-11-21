#!/usr/bin/env python

import rospy
import rospkg

from tree_transform.srv import tree_transformService, tree_transformServiceResponse # importing the srv file top and bottom
from data_fusion_node.msg import PoseArrayId # importing custom message of Grapes pose arrays
from apriltag_ros.msg import AprilTagDetectionArray # importing the custom message of apriltags

def get_tree_transf_client():
    rospy.wait_for_service('tree_transf_srv') # wait for service to be available
    try:
        rospy.logwarn("here") # service is available
        client_handle = rospy.ServiceProxy('tree_transf_srv', tree_transformService) # calling the service that we want to call
     	# wait topic
        tag=rospy.wait_for_message("/tag_detections", AprilTagDetectionArray) # we wait for AprilTagDetectionArray message
        detections=rospy.wait_for_message("/poses_pfc", PoseArrayId) # we wait for the PoseArrayID message

        #rospy.logwarn(detections.poses)
        rospy.logwarn("here 1")
        response1 = client_handle(tag,detections.poses) # passing the two received messages to the service and saving the response
        rospy.logwarn("here 2")
        rospy.logwarn(response1) 
        return response1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('tree_transform_client') # initializing client
    get_tree_transf_client()
