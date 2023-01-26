#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd
import rospkg
import os
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import PointStamped
from tree_transform.msg import PoseStampedArray,PoseStampedCustom # importing custom message PoseStampedArray
from tree_transform.srv import tree_transformService, tree_transformServiceResponse # importing the srv file top and bottom
from tree_transform.srv import database_manager_load, database_manager_loadResponse # importing the srv file top and bottom
from tree_transform.srv import database_manager_clean, database_manager_cleanResponse # importing the srv file top and bottom
from tree_transform.srv import database_manager_update, database_manager_updateResponse # importing the srv file top and bottom
from tree_transform.srv import database_manager_upload, database_manager_uploadResponse # importing the srv file top and bottom
from tree_transform.srv import database_manager_get_poses, database_manager_get_posesResponse # importing the srv file top and bottom
from data_fusion_node.msg import PoseArrayId # importing custom message of Grapes pose arrays
from apriltag_ros.msg import AprilTagDetectionArray # importing the custom message of apriltags
import time

class Database_handler(object):
    def __init__(self):
        rospy.loginfo("Initialized database handler")

    def load_database(self,req):
        #self.df = pd.read_csv("tree_transform/dataframe_grapes_test_empty.csv") # decomment this to delete the csv
        status=Int8()
        rospack = rospkg.RosPack()
        self.cwd_path = rospack.get_path('tree_transform')

        self.cwd_path=os.path.join(self.cwd_path,req.csv_name.data)
        rospy.logwarn(self.cwd_path)
        try:
            self.df = pd.read_csv(self.cwd_path,index_col=0)
            print("showing current database :")
            print(self.df.to_string())
            status.data=0
        except:
            status.data=1
        return database_manager_loadResponse(status=status)

    def update_database(self,req):
        rospy.loginfo("updating database")
        status=Int8()
        try:
            poses_detected=req.poses
            for i in range(len(poses_detected.poses)):

                id= poses_detected.poses[i].tag
                dist= poses_detected.poses[i].dist
                tag_id = poses_detected.poses[i].pose.header.frame_id
                x = poses_detected.poses[i].pose.pose.position.x
                y = poses_detected.poses[i].pose.pose.position.y
                z = poses_detected.poses[i].pose.pose.position.z
                orx = poses_detected.poses[i].pose.pose.orientation.x
                ory = poses_detected.poses[i].pose.pose.orientation.y
                orz = poses_detected.poses[i].pose.pose.orientation.z
                orw = poses_detected.poses[i].pose.pose.orientation.w
                time = poses_detected.poses[i].pose.header.stamp.secs # used when removing a grape.

                self.df_pose = pd.DataFrame([[tag_id, id, dist, x, y, z, orx, ory, orz, orw, time]],
                        columns=['tag_Id', 'Id','dist', 'positionx', 'positiony', 'positionz', 'orientationx', 'orientationy', 'orientationz', 'orientationw','time'],
                        index=[id])
                self.df=pd.concat([self.df, self.df_pose])
                #print("single pose wrt to tag")
                #print(self.df_pose.to_string())
            print("showing current database :")
            print(self.df.to_string())
            self.df_updated = self.df.sort_index()
            status.data=0
        except:
            status.data=1
        return database_manager_updateResponse(status=status)
        #print("updated dataframe")
        #print(self.df_updated.to_string())

    def clean_database(self,req): # cleans database from double identification of same grape
    # get a vector of all the indices contained in the df ( all the tags that we found)
    # remove redundancies from vector
    # check the distance of every id found
    # remove grapes with distance too close ( maybe use vector diff)
    # self.df_clean = df[["Id"]]
        status=Int8()
        try:
            self.df_clean = pd.DataFrame() # taking the first row of updated df
            #print(" this is first row \n")
            #print(self.df_clean.to_string())
            self.Id_array = np.array(self.df_updated[["Id"]]).T
            self.Id_array = np.unique(self.Id_array) # getting unique values
            #print("this is the id_array")
            #print(self.Id_array)
            for idx, x in enumerate(self.Id_array): # here we have the idx and the values inside the vector of id
                #print("this is x")
                #print(x)
                #print("this is idx")
                #print(idx)
                #print("try error 1")
                #print((self.df_updated['Id'] == x).to_string())
                self.df_tree = self.df_updated.loc[self.df_updated['Id'] == x] # getting df components regarding tree Id
                #print("this is the the single tree df")
                #print(self.df_tree)
                self.df_tree.sort_values(by=['dist'],inplace=True) # ordering by distance
                #print("this is the the single tree df sorted by distance")
                #print(self.df_tree)
                self.df_tree_dist = self.df_tree[["dist"]] # get column of dist
                #print("this is the the single tree df only distances")
                #print(self.df_tree_dist)
                # create column of difference of current distance and next distance
                self.df_tree_dist['ddist'] = self.df_tree_dist['dist'] - self.df_tree_dist['dist'].shift(periods=1, fill_value=100)
                #print("adding column ddist")
                #print(self.df_tree_dist)
                # takes the cleaned tree
                distance_param=req.distance.data
                self.df_tree_clean=self.df_tree.loc[(self.df_tree_dist["ddist"]<-distance_param) | (self.df_tree_dist["ddist"]>distance_param)]
                #print("removing the distances")
                #print(self.df_tree_clean)
                self.df_clean=pd.concat([self.df_clean, self.df_tree_clean])
            status.data=0
        except:
            status.data=1
        return database_manager_cleanResponse(status=status)
        #print("cleaned dataframe")
        #print(self.df_clean.to_string())


    def upload_database(self,req):
        #print("uploading dataframe")
        status=Int8()
        try:
            self.df_clean.to_csv(self.cwd_path) # sill to be tested
            status.data=0
        except:
            status.data=1
        return database_manager_uploadResponse(status=status)
    def get_poses(self,req):
        status=Int8()
        try:
            df = self.df_clean
           # datatypes = {'Id': float,'dist': float,
        	#	'positionx': float,'positiony': float,'positionz': float,
        	#	'orientationx': float,'orientationy': float,'orientationz': float,'orientationw': float}

            df = df.to_numpy()
            #print("this is df")
            #print(df)
            poses_df = PoseStampedArray()
            for i in range(df.shape[0]):
                pose_df = PoseStampedCustom()
                print(df[i,1])

                pose_df.tag = df[i,1]
                pose_df.dist = df[i,2]
                pose_df.pose.header.stamp.secs = df[i,10]
                pose_df.pose.header.frame_id = df[i,0]
                pose_df.pose.pose.position.x = df[i,3]
                pose_df.pose.pose.position.y = df[i,4]
                pose_df.pose.pose.position.z = df[i,5]
                pose_df.pose.pose.orientation.x = df[i,6]
                pose_df.pose.pose.orientation.y = df[i,7]
                pose_df.pose.pose.orientation.z = df[i,8]
                pose_df.pose.pose.orientation.w = df[i,9]
                poses_df.poses.append(pose_df)
            status.data=0
        except:
            status.data=1
        return database_manager_get_posesResponse(status=status,poses=poses_df)


def load_server(handler):
    s_load = rospy.Service('load_srv',  database_manager_load, handler.load_database)
def update_server(handler):
    s_update = rospy.Service('update_srv',  database_manager_update, handler.update_database)
def clean_server(handler):
    s_clean = rospy.Service('clean_srv',  database_manager_clean, handler.clean_database)
def upload_server(handler):
    s_upload = rospy.Service('upload_srv',  database_manager_upload, handler.upload_database)
def get_poses_server(handler):
    s_get_poses = rospy.Service('get_poses_srv',  database_manager_get_poses, handler.get_poses)

if __name__ == '__main__':
    rospy.init_node('database_handler', anonymous=True)
    handler = Database_handler()
    # you could name this function
    load_server(handler)
    update_server(handler)
    clean_server(handler)
    upload_server(handler)
    get_poses_server(handler)
    rospy.spin()
