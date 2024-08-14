#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped



def callback_amcl(amcl):
    rospy.loginfo(f" AMCL Pos {amcl.position} ")
    rospy.loginfo(f" AMCL Ori {amcl.orientation} ")
    
def callback_ground_truth(ground_truth):
    rospy.loginfo(f"Ground Truth Pos {ground_truth.pose.position} ")
    rospy.loginfo(f"Ground Truth Ori {ground_truth.pose.orientation} ")
    

def listener():
    rospy.init_node('listener_mur620d_pose')
    rospy.Subscriber("/mur620d/robot_pose",Pose, callback_amcl)
    rospy.Subscriber("/qualisys_map/mur620d/pose",PoseStamped, callback_ground_truth)

    
    
    rospy.spin()

if __name__ == '__main__':
    listener()