#!/usr/bin/env python
# pub-sub node to transform detected object into tree reference frame

import rospkg
import rospy
import tf as transf
from geometry_msgs.msg import PoseArray
from tree_transform.srv import tree_transformResponse


grapes_wrt_tag = PoseArray()

def transform_req(req):
    # AprilTagDetectionArray :custom message with header + detections,
    # detections is a custom message with id , size, pose
    apriltag = req.tag
    grapes_wrt_camera = req.detection # geometry_msgs/PoseArray
    listener = transf.TransformListener()
    listener.waitForTransform(apriltag.header.frame_id,grapes_wrt_camera.header.frame_id,grapes_wrt_camera.header.stamp,rospy.Duration(3.0))
    grapes_wrt_tag.header.stamp = listener.getLatestCommonTime(apriltag.header.frame_id,grapes_wrt_camera.header.frame_id)
    grapes_wrt_tag.header.frame_id = apriltag.header.frame_id

    for i in range(grapes_wrt_camera.poses): # grapes_wrt_camera.header.frame_id[i]
        grapes_wrt_tag.poses.append(listener.transformPose(apriltag.header.frame_id,grapes_wrt_camera.poses[i]))


    return tree_transformResponse(grapes_wrt_tag=grapes_wrt_tag)

def tree_transf_server():
    rospy.init_node('tree_transform_node')
    s = rospy.Service('tree_transf_srv', tree_transformResponse, transform_req)
    rospy.spin()

if __name__ == '__main__':
    tree_transf_server()
