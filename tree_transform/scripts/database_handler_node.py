#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd

from std_msgs.msg import String
from geometry_msgs import PointStamped
from tree_transform.msg import PoseStampedArray,PoseStampedCustom # importing custom message PoseStampedArray
from tree_transform.srv import tree_transformService, tree_transformServiceResponse # importing the srv file top and bottom
from data_fusion_node.msg import PoseArrayId # importing custom message of Grapes pose arrays
from apriltag_ros.msg import AprilTagDetectionArray # importing the custom message of apriltags
import time

class Database_handler(object):
    def __init__(self):
        rospy.loginfo("Initialized database handler")

    def load_database(self):
        #self.df = pd.read_csv("tree_transform/dataframe_grapes_test_empty.csv") # decomment this to delete the csv
        self.df = pd.read_csv("tree_transform/poses_dataframe.csv",index_col=0)
	print("showing current database :")
        print(self.df.to_string())

    def update_database(self,poses_detected):
        rospy.loginfo("updating database")
        for i in range(len(poses_detected.detection_res.poses)):

            id= poses_detected.detection_res.poses[i].tag
            dist= poses_detected.detection_res.poses[i].dist
            tag_id = poses_detected.detection_res.poses[i].pose.header.frame_id
            x = poses_detected.detection_res.poses[i].pose.pose.position.x
            y = poses_detected.detection_res.poses[i].pose.pose.position.y
            z = poses_detected.detection_res.poses[i].pose.pose.position.z
            orx = poses_detected.detection_res.poses[i].pose.pose.orientation.x
            ory = poses_detected.detection_res.poses[i].pose.pose.orientation.y
            orz = poses_detected.detection_res.poses[i].pose.pose.orientation.z
            orw = poses_detected.detection_res.poses[i].pose.pose.orientation.w
            time = poses_detected.detection_res.poses[i].pose.header.stamp.secs # used when removing a grape.

            self.df_pose = pd.DataFrame([[tag_id, id, dist, x, y, z, orx, ory, orz, orw, time]],
                    columns=['tag_Id', 'Id','dist', 'positionx', 'positiony', 'positionz', 'orientationx', 'orientationy', 'orientationz', 'orientationw','time'],
                    index=[id])
            self.df=pd.concat([self.df, self.df_pose])
            #print("single pose wrt to tag")
            #print(self.df_pose.to_string())

        self.df_updated = self.df.sort_index()
        #print("updated dataframe")
        #print(self.df_updated.to_string())

    def clean_database(self): # cleans database from double identification of same grape
    # get a vector of all the indices contained in the df ( all the tags that we found)
    # remove redundancies from vector
    # check the distance of every id found
    # remove grapes with distance too close ( maybe use vector diff)
    # self.df_clean = df[["Id"]]
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
            self.df_tree_clean=self.df_tree.loc[(self.df_tree_dist["ddist"]<-0.15) | (self.df_tree_dist["ddist"]>0.15)]
            #print("removing the distances")
            #print(self.df_tree_clean)
            self.df_clean=pd.concat([self.df_clean, self.df_tree_clean])

        #print("cleaned dataframe")
        #print(self.df_clean.to_string())


    def upload_database(self):
        #print("uploading dataframe")
        self.df_clean.to_csv(r'tree_transform/poses_dataframe.csv') # sill to be tested

# def subscriber_callback(data): ############ ask ivan!
#
#     # print the actual message in its raw format
#     rospy.loginfo("Here's what was subscribed:")
#     rospy.loginfo(data)
#     # otherwise simply print a convenient message on the terminal
#     print('subscriber worked correctly')


def get_tree_transf_client(handler):
    print("searching service")
    rospy.wait_for_service('tree_transf_srv',5.0) # wait for service to be available, frquence of node is set here!!
    try:
        rospy.logwarn("here") # service is available
        client_handle = rospy.ServiceProxy('tree_transf_srv', tree_transformService) # calling the service that we want to call
        detections=rospy.wait_for_message("/poses_pfc", PoseArrayId) # we wait for the PoseArrayID message
        tag=rospy.wait_for_message("/tag_detections", AprilTagDetectionArray) # we wait for AprilTagDetectionArray message

        #rospy.logwarn(detections.poses)
        # rospy.logwarn("here 1")
        response1 = client_handle(tag,detections.poses) # passing the two received messages to the service and saving the response
        # rospy.logwarn("here 2")
        # rospy.logwarn(response1)
        rospy.loginfo("Service called correctly")
	#handler.update_database(poses_detected)
	handler.update_database(response1)
    	handler.clean_database()
   	handler.upload_database()
        return response1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def publish_poses():
    df = pd.read_csv("tree_transform/poses_dataframe.csv",index_col=0)
   # datatypes = {'Id': float,'dist': float,
	#	'positionx': float,'positiony': float,'positionz': float,
	#	'orientationx': float,'orientationy': float,'orientationz': float,'orientationw': float}

    df = df.to_numpy()
    #print("this is df")
    #print(df)
    poses_df = PoseStampedArray()
    for i in range(df.shape[0]):
        pose_df = PoseStampedCustom()
	#print(i)

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

    poses_pub.publish(poses_df)

    def get_point(tag_Id):
        # recieve tag number
        #tag_string= "tag_"+str(tag_Id)
        point = PointStamped()
        df_get_point= pd.read_csv("tree_transform/poses_dataframe.csv",index_col=0) # gets df
        # gets df of tree Id ( already ordered from distmin to distmax)
        df_tree_get_point = df_get_point.loc[df_get_point['Id'] == tag_Id]
        print(df_tree_get_point.to_string())
        point = PointStamped()
        i=0;
        # if the time value of the first one is zero (dummy grape), we pass to the second
        if df_tree_get_point[i,10] == 0:
            i=1

        # point.tag = df_tree_get_point[i,1]
        # point.dist = df_tree_get_point[i,2]
        point.pose.header.stamp.secs = df_tree_get_point[i,10]
        point.pose.header.frame_id = df_tree_get_point[i,0]
        point.pose.pose.position.x = df_tree_get_point[i,3]
        point.pose.pose.position.y = df_tree_get_point[i,4]
        point.pose.pose.position.z = df_tree_get_point[i,5]
        # point.pose.pose.orientation.x = df_tree_get_point[i,6]
        # point.pose.pose.orientation.y = df_tree_get_point[i,7]
        # point.pose.pose.orientation.z = df_tree_get_point[i,8]
        # point.pose.pose.orientation.w = df_tree_get_point[i,9]

        return point




def main():
    while not rospy.is_shutdown():

    # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
	 handler.load_database()
	 try:
   	 	poses_detected = get_tree_transf_client(handler)
    # rospy.loginfo("these are the poses")
    # rospy.loginfo(poses_detected)
	 except:

   	 	publish_poses()
    # poses = rospy.Subscriber("/PoseStampedArray", PoseStampedArray, subscriber_callback)


if __name__ == '__main__':

    # you could name this function
    try:
	rospy.init_node('database_handler', anonymous=True)
    	handler = Database_handler()
        poses_pub = rospy.Publisher('database_poses', PoseStampedArray, queue_size=10)
        main()
    except rospy.ROSInterruptException:
        pass
