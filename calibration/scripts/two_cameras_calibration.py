#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import quaternion
from geometry_msgs.msg import Pose
from apriltag_ros.msg import AprilTagDetectionArray # importing custom message AprilTagDetectionArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler,quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np

global i
global q_vect
global p_vect

def PoseStamped_2_mat(p):
    q = p.orientation
    pos = p.position
    T = quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = np.array([pos.x,pos.y,pos.z])
    return T

def Mat_2_posestamped(m,f_id="test"):
    q = quaternion_from_matrix(m)
    p = PoseStamped(header = Header(frame_id=f_id), #robot.get_planning_frame()
                    pose=Pose(position=Point(*m[:3,3]),
                    orientation=Quaternion(*q)))
    return p

def T_inv(T_in):
    R_in = T_in[:3,:3]
    t_in = T_in[:3,[-1]]
    R_out = R_in.T
    t_out = -np.matmul(R_out,t_in)
    return np.vstack((np.hstack((R_out,t_out)),np.array([0, 0, 0, 1])))


def callback_two_images(tag_det_cam_1, tag_det_cam_2):
    # print(" inside callback ")
    global i

    tag_10=tag_det_cam_1.detections[0].pose.pose.pose
    tag_20=tag_det_cam_2.detections[0].pose.pose.pose

    A_cam1_tag=PoseStamped_2_mat(tag_10)
    A_cam2_tag=PoseStamped_2_mat(tag_20)
    A_tag_cam2=T_inv(A_cam2_tag)
    A_cam1_cam2=np.matmul(A_cam1_tag, A_tag_cam2)

    q_vect.append(quaternion_from_matrix(A_cam1_cam2))
    p_vect.append(A_cam1_cam2[:3,3])

    if i%100==0:
        transform=Mat_2_posestamped(A_cam1_cam2)
        print("*************************************")
        print(transform)
        # print("This is p_vect")
        # print(p_vect)
        mean_x= np.mean([item[0] for item in p_vect])
        mean_y= np.mean([item[1] for item in p_vect])
        mean_z= np.mean([item[2] for item in p_vect])

        mean_xq= np.mean([item[0] for item in q_vect])
        mean_yq= np.mean([item[1] for item in q_vect])
        mean_zq= np.mean([item[2] for item in q_vect])
        mean_wq= np.mean([item[3] for item in q_vect])
        # print("This is q_vect")
        # print(q_vect)
        print("this is vector")
        print(mean_x)
        print(mean_y)
        print(mean_z)
        print("this are quaternions")

        print(mean_xq)
        print(mean_yq)
        print(mean_zq)
        print(mean_wq)


    i+=1


if __name__ == '__main__':

    rospy.init_node('two_cameras_calib_node', anonymous=True)
    i=0
    q_vect=[]
    p_vect=[]

    tag_det_cam_1 = message_filters.Subscriber("/cam_1/color/tag_detections", AprilTagDetectionArray)
    tag_det_cam_2 = message_filters.Subscriber("/cam_2/color/tag_detections", AprilTagDetectionArray)

    print(" entering message filters part ")
    ts = message_filters.ApproximateTimeSynchronizer([tag_det_cam_1, tag_det_cam_2], 1,0.1)
    print("  message filters recieved ")
    ts.registerCallback(callback_two_images)

    rospy.spin()
