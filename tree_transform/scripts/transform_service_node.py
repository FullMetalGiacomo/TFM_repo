#!/usr/bin/env python2
# pub-sub node to transform detected object into tree reference frame


import tf as transf

# import sys
# print(sys.version)
# tree_transform
from tree_transform.srv import tree_transformService, tree_transformServiceResponse # importing srv up and bottom

from data_fusion_node.msg import PoseArrayId # importing custom message PoseArrayID
from apriltag_ros.msg import AprilTagDetectionArray # importing custom message AprilTagDetectionArray
from tree_transform.msg import PoseStampedArray,PoseStampedCustom # importing custom message PoseStampedArray
from geometry_msgs.msg import PoseArray # importing custom message PoseArray
from geometry_msgs.msg import PoseStamped  # importing custom message PoseStamped
import rospkg
import rospy
import numpy as np


# global detect_pub # decomment detect_pub for publishing PoseStampedArray

def transform_req(req): # here we call the handler
    # grapes_wrt_tag_t = PoseArray()
    rospy.logwarn("Service request recieved!")
    try:
        grapes_wrt_tag_t = PoseStampedArray()
        # AprilTagDetectionArray :custom message with header + detections,
        # detections is a custom message with id , size, pose

        apriltag = req.tag # reading the tag AprilTagDetectionArray msg
        grapes_wrt_camera = req.detection # reading the geometry_msgs/PoseArray from detection
        rospy.logwarn(apriltag.header.frame_id) # prints the frame id of apriltags
        rospy.logwarn(grapes_wrt_camera) # prints the grapes_wrt_camera
        listener = transf.TransformListener() # creating the transform listener
        current_time = rospy.Time.now()
        # waiting for the transform between "tag_0" ( "tag_i") and the camera frame apriltag.detections[i].id[0]
        ################################################################################################ my risky attempt

        dist=np.zeros((len(grapes_wrt_camera.poses),len(apriltag.detections))) # array containing rows of # grapes and cols of # tags
        for i in range(len(apriltag.detections)):
            x_tag = apriltag.detections[i].pose.pose.pose.position.x
            y_tag = apriltag.detections[i].pose.pose.pose.position.y
            z_tag = apriltag.detections[i].pose.pose.pose.position.z
            for j in range(len(grapes_wrt_camera.poses)):
                x_grape = grapes_wrt_camera.poses[j].position.x
                y_grape = grapes_wrt_camera.poses[j].position.y
                z_grape = grapes_wrt_camera.poses[j].position.z
                dist[j,i]=np.sqrt((x_grape - x_tag)**2 + (y_grape - y_tag)**2 + (z_grape - z_tag)**2) # building distance matrix

        idx_tag=np.argmin(dist, axis=1) # finds min index by rows "contains the tags id"
        #idx is a vector composed like this [0 1 0 1 2 2 1 0] where the value corrisponds to the tag and the idx position to the grape.
        #d_min=dist.min(axis=1)# find minumum distances " contains the minumum dist"
        for k in range(len(apriltag.detections)):
            for t in range(len(grapes_wrt_camera.poses)):
       	        #rospy.logwarn(t)
                grapes_wrt_tag_s = PoseStampedCustom()
                # grapes_wrt_tag_s.header.stamp = listener.getLatestCommonTime("tag_"+str(apriltag.detections[k].id[0]),grapes_wrt_camera.header.frame_id) # taking the time
                grapes_wrt_camera.header.stamp = apriltag.detections[k].pose.header.stamp
                listener.waitForTransform("tag_"+str(apriltag.detections[k].id[0]),grapes_wrt_camera.header.frame_id,grapes_wrt_tag_s.pose.header.stamp,rospy.Duration(3.0))
                grapes_wrt_tag_s.tag = apriltag.detections[k].id[0]
                grapes_wrt_tag_s.dist = dist[t,k]
                grapes_wrt_tag_s.pose.header.stamp = listener.getLatestCommonTime("tag_"+str(apriltag.detections[k].id[0]),grapes_wrt_camera.header.frame_id) # taking the time
                grapes_wrt_tag_s.pose.header.frame_id = grapes_wrt_camera.header.frame_id # creating the frame ID
                grapes_wrt_tag_s.pose.pose=grapes_wrt_camera.poses[t] # taking each pose
                grapes_wrt_tag_s.pose=listener.transformPose("tag_"+str(apriltag.detections[k].id[0]),grapes_wrt_tag_s.pose) # taking each pose
                grapes_wrt_tag_s.pose.header.stamp = current_time # changing time to get univoque marking of grape
                print(current_time)
                # grapes_wrt_tag_t.poses.append(listener.transformPose("tag_"+str(apriltag.detections[k].id[0]),grapes_wrt_tag_s).pose) # transformming and appending the new pose
                grapes_wrt_tag_t.poses.append(grapes_wrt_tag_s) # transformming and appending the new pose
     # transformming and appending the new pose
        # detect_pub.publish(grapes_wrt_tag_t)
        rospy.logwarn(grapes_wrt_tag_t)

        ################################################################################################
        # listener.waitForTransform("tag_"+str(apriltag.detections[0].id[0]),grapes_wrt_camera.header.frame_id,grapes_wrt_camera.header.stamp,rospy.Duration(3.0))
        # grapes_wrt_tag_t.header.stamp = listener.getLatestCommonTime("tag_"+str(apriltag.detections[0].id[0]),grapes_wrt_camera.header.frame_id) # taking the time
        # grapes_wrt_tag_t.header.frame_id = "tag_"+str(apriltag.detections[0].id[0])# creating the frame ID
        #
        # for i in range(len(grapes_wrt_camera.poses)): # grapes_wrt_camera.header.frame_id[i]
        #     grapes_wrt_tag_s = PoseStamped()
        #     rospy.logwarn(grapes_wrt_camera.poses[i])
        #     grapes_wrt_tag_s.header=grapes_wrt_camera.header
        #     grapes_wrt_tag_s.pose=grapes_wrt_camera.poses[i] # taking each pose
        #     grapes_wrt_tag_t.poses.append(listener.transformPose("tag_"+str(apriltag.detections[0].id[0]),grapes_wrt_tag_s).pose) # transformming and appending the new pose
        #
        # rospy.logwarn(grapes_wrt_tag_t)
        return  tree_transformServiceResponse(detection_res=grapes_wrt_tag_t) # assigning the constructed PoseArray
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


def tree_transf_server():
    rospy.init_node('tree_transform_node', anonymous=True) # initialize the service node
    rospy.loginfo("tree_transform_srv node ready")
    s = rospy.Service('tree_transf_srv',  tree_transformService, transform_req)
    rospy.spin()

if __name__ == '__main__':
    # detect_pub = rospy.Publisher('/PoseStampedArray',  PoseStampedArray, queue_size=10)
    tree_transf_server()
