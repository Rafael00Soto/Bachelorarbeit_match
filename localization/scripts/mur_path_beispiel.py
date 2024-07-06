#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import tf
import numpy as np
import math


class Create_mur_path():
    def config(self):
        self.start_pose.position.x = 0.0 #0.0 Just for quick tests
        self.start_pose.position.y = 0.0 
        self.point_per_meter = 100
        self.distance = 3 
        self.dt = 20/self.point_per_meter # seconds between points

    def main(self):
        rospy.init_node("create_mur_path_node")

        self.start_pose = Pose()
        self.config()
        rospy.sleep(0.1)
        self.path_pub= rospy.Publisher("/mur_path", Path, queue_size=1, latch=True)# Como se llama el topic al que tenemos que publicar?

        #Path between two points
        mur_path = Path()
        mur_path.header.frame_id = "map"
        #Dummy path for tests
        for i in range(0,int(self.distance*self.point_per_meter)):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.header.seq = i
            #Straight Line
            next_pose=PoseStamped()
            next_pose.pose.position.y=pose.pose.position.y
            next_pose.pose.position.x= pose.pose.position.x + (1/self.point_per_meter)*i

            orientation = math.atan2(next_pose.pose.position.y - pose.pose.position.y, next_pose.pose.position.x - pose.pose.position.x)
            q = tf.transformations.quaternion_from_euler(0, 0, orientation)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            mur_path.poses.append(next_pose)

           
            



        rospy.sleep(1.1)
        rospy.loginfo("Publishing MurPath")
        self.path_pub.publish(mur_path)

        rospy.loginfo("Paths published")
        rospy.sleep(30)        

        
if __name__=="__main__":
    exe = Create_mur_path()
    exe.main()