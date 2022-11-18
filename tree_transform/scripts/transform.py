#!/usr/bin/env python2
# pub-sub node to transform detected object into tree reference frame


import tf as transf

import sys
print(sys.version)
# tree_transform
from tree_transform.srv import tree_transformService, tree_transformServiceResponse

from data_fusion_node.msg import PoseArrayId
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
import rospkg
import rospy




def transform_req(req):
    rospy.logwarn("AQUI 0")
    grapes_wrt_tag_t = PoseArray()
    # AprilTagDetectionArray :custom message with header + detections,
    # detections is a custom message with id , size, pose

    apriltag = req.tag
    grapes_wrt_camera = req.detection # geometry_msgs/PoseArray
    rospy.logwarn(apriltag.header.frame_id)
    rospy.logwarn(grapes_wrt_camera)
    listener = transf.TransformListener()
    listener.waitForTransform("tag_"+str(apriltag.detections[0].id[0]),grapes_wrt_camera.header.frame_id,grapes_wrt_camera.header.stamp,rospy.Duration(3.0))
    grapes_wrt_tag_t.header.stamp = listener.getLatestCommonTime("tag_"+str(apriltag.detections[0].id[0]),grapes_wrt_camera.header.frame_id)
    grapes_wrt_tag_t.header.frame_id = "tag_"+str(apriltag.detections[0].id[0])

    for i in range(len(grapes_wrt_camera.poses)): # grapes_wrt_camera.header.frame_id[i]
        grapes_wrt_tag_s = PoseStamped()
        rospy.logwarn(grapes_wrt_camera.poses[i])
        grapes_wrt_tag_s.header=grapes_wrt_camera.header
        grapes_wrt_tag_s.pose=grapes_wrt_camera.poses[i]
        grapes_wrt_tag_t.poses.append(listener.transformPose("tag_"+str(apriltag.detections[0].id[0]),grapes_wrt_tag_s).pose)

    rospy.logwarn(grapes_wrt_tag_t)
    return  tree_transformServiceResponse(detection_res=grapes_wrt_tag_t)

def tree_transf_server():
    rospy.init_node('tree_transform_node', anonymous=True)
    rospy.logwarn("AQUI Marco")
    s = rospy.Service('tree_transf_srv',  tree_transformService, transform_req)
    rospy.spin()

if __name__ == '__main__':
    tree_transf_server()
