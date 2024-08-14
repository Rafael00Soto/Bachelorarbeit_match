#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    
    pub = rospy.Publisher('/mur620d/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('mur620d_simple', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp= rospy.Time.now()
        pose_msg.header.frame_id="map"
        
        pose_msg.pose.position.x= 36.5
        pose_msg.pose.position.y= 34.0
        pose_msg.pose.position.z= 0.0
        
        pose_msg.pose.orientation.x= 0.0
        pose_msg.pose.orientation.y= 0.0
        pose_msg.pose.orientation.z= 0.0
        pose_msg.pose.orientation.w= 1.0
        
        pub.publish(pose_msg)
        
        rate.sleep()