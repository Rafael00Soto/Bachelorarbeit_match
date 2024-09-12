#!/usr/bin/env python3
import rospy
import actionlib
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


#roslaunch mur_launch_hardware mur620d.launch launch_ur_r:=false launch_ur_l:=false
#source catkin_ws_amcl/devel/setup.bash
#systemctl start mocap
#catkin_ws/src/match_mocap/mocap_toolbox/scripts
#roslaunch mocap_toolbox transformation_mocap_amcl.launch
if __name__ == "__main__":
    
    rospy.init_node("send_mur620d_to_pose")
    rospy.sleep(0.1)
    client = actionlib.SimpleActionClient('/mur620d/move_base', MoveBaseAction)  
    rospy.loginfo("Waiting for move base flex server") 
    client.wait_for_server()
    goal_client = MoveBaseGoal()
    rospy.sleep(1.0)

    # Desired position
    goal = PoseStamped()
    x_position = 31.3
    y_position = 29.7
    angle_degrees = 0.0
    angle_radians = math.radians(angle_degrees)
    q = tf.transformations.quaternion_from_euler(0, 0, angle_radians)

    goal_client.target_pose.header.frame_id = 'map' 
    goal_client.target_pose.pose.orientation.x = q[0]
    goal_client.target_pose.pose.orientation.y = q[1]
    goal_client.target_pose.pose.orientation.z = q[2]
    goal_client.target_pose.pose.orientation.w = q[3]
    goal_client.target_pose.pose.position.x = x_position
    goal_client.target_pose.pose.position.y = y_position

    rospy.sleep(1.0)
    client.send_goal(goal_client)
    rospy.loginfo("Goal sent, waiting for result")

    state = client.wait_for_result()

    if state:
        result_state = client.get_state()
        if result_state == 3:
            rospy.loginfo("Point reached")
        elif result_state == 4:
            rospy.loginfo("Point aborted")
        else:
            rospy.loginfo("Unknown state: %d", result_state)
    else:
        rospy.loginfo("Action did not finish before the timeout")

    rospy.loginfo("Shutting down node")

    
    
    
    
    


    
