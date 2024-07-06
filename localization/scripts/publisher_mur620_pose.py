#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf


if __name__ == '__main__':
    rospy.init_node("define_goal_pose_node", anonymous=True)
    rospy.sleep(0.1)
    publisher=rospy.Publisher("/mur620/move_base_simple/goal", PoseStamped, queue_size=10)

    goal = PoseStamped()
    goal.header.stamp= rospy.Time.now()
    goal.header.frame_id= "map"
    goal.header.seq= 1
    q = tf.transformations.quaternion_from_euler(0, 0, (45))
    goal.pose.orientation.x=q[0]
    goal.pose.orientation.y=q[1]
    goal.pose.orientation.z=q[2]
    goal.pose.orientation.w=q[3]
    goal.pose.position.x=-7.0
    goal.pose.position.y=4.0
    publisher.publish(goal)
    rospy.sleep(1.0)
    rospy.loginfo("Goal published")