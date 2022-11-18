#!/usr/bin/env python2
# pub-sub node to transform detected object into tree reference frame

import rospkg
import rospy
import tf as transf
from data_fusion_node.msg import PoseArrayId
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from tree_transform.srv import tree_transformService, tree_transformResponse




def transform_req(req):
    rospy.logwarn("AQUI 0")
    grapes_wrt_tag_t = PoseArray()
    # AprilTagDetectionArray :custom message with header + detections,
    # detections is a custom message with id , size, pose

    apriltag = req.tag
    grapes_wrt_camera = req.detection # geometry_msgs/PoseArray
    rospy.logwarn("AQUI 1")
    listener = transf.TransformListener()
    listener.waitForTransform(apriltag.header.frame_id,grapes_wrt_camera.header.frame_id,grapes_wrt_camera.header.stamp,rospy.Duration(3.0))
    grapes_wrt_tag_t.header.stamp = listener.getLatestCommonTime(apriltag.header.frame_id,grapes_wrt_camera.header.frame_id)
    grapes_wrt_tag_t.header.frame_id = apriltag.header.frame_id

    for i in range(grapes_wrt_camera.poses): # grapes_wrt_camera.header.frame_id[i]
        grapes_wrt_tag_t.poses.append(listener.transformPose(apriltag.header.frame_id,grapes_wrt_camera.poses[i]))

    rospy.logwarn("AQUI 2")
    return tree_transformResponse(grapes_wrt_tag=grapes_wrt_tag_t)

def tree_transf_server():
    rospy.init_node('tree_transform_node', anonymous=True)
    rospy.logwarn("AQUI Marco")
    s = rospy.Service('tree_transf_srv', tree_transformService, transform_req)
    rospy.spin()

if __name__ == '__main__':
    tree_transf_server()
