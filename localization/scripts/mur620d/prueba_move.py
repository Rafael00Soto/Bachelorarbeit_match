#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import time



def move(velocity_publisher, speed, distance):

    velocity_message= Twist ()

    global x, y
    x0=x
    y0=y 

    
    velocity_message.linear.x= abs(speed)

    distance_moved= 0.0
    loop_rate = rospy.Rate(10)
    rospy.loginfo("Robot moves forwards")
    while True:
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved= abs(  math.sqrt(((x-x0)**2)+(y-y0)**2))
        if not (distance_moved<distance):
            rospy.loginfo("Desired distance reached")
            break

    velocity_message.linear.x=0.1
    velocity_publisher.publish(velocity_message)

def poseCallback(pose_message):
    global x, y, q
    x= pose_message.position.x
    y= pose_message.position.y
    q=pose_message.orientation 

if __name__ == '__main__':
    try:
        rospy.init_node('move_node_tutorial', anonymous=True)

        cmd_vel_topic='mur620/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        position_topic = "mur620/mir_pose_simple"
        pose_subscriber= rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)
        distance_to_move=4.0
        move(velocity_publisher,1.0, distance_to_move)
    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')