#! /usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from odf.opendocument import OpenDocumentSpreadsheet
from odf.table import Table, TableRow, TableCell
from odf.text import P
import math




def callback_ground_truth(pose_ground_truth):
     global ground_truth
     ground_truth= pose_ground_truth

def callback_amcl(pose_amcl):
     global amcl
     amcl=pose_amcl


if __name__=="__main__":
    
    rospy.init_node("collect_data_goal", anonymous=True)
    rospy.sleep(0.1)
    client = actionlib.SimpleActionClient('mur620d/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()
    goal_client=MoveBaseGoal()

    rospy.sleep(1.0)
    listener1=rospy.Subscriber("/mur620d/robot_pose",Pose, callback_amcl)
    listener2=rospy.Subscriber("qualisys_map/mur620d/pose",PoseStamped, callback_ground_truth)

    #Desired positions
    config=[]
    config.append([33.8 ,30.11 , 180])
    config.append([25.6 ,29.8, 25])  
    config.append([34.0 ,33.0, 105])
    config.append([25.3 ,32.7, 300])
    config.append([29.2, 28.0, 15])

    doc = OpenDocumentSpreadsheet()
    table = Table(name="Table1")
    
    columns = ["Cycle","Configuration number","Amcl Pose x", "Amcl Pose y", "Amcl Pose Orientation", "Ground Truth x", "Ground Truth y", "Ground Truth Orientation"]
    header_row = TableRow()
    for column in columns:
        cell = TableCell()
        cell.addElement(P(text=str(column)))
        header_row.addElement(cell)
    table.addElement(header_row)
    
    goal = PoseStamped()
    for j in range(1,21):
          for i in range (0,5):
               angle_degrees= config[i][2]
               angle_radians=math.radians(angle_degrees)
               q = tf.transformations.quaternion_from_euler(0, 0, angle_radians)
               rospy.sleep(0.1)

               goal_client.target_pose.header.frame_id = 'map' 
               goal_client.target_pose.pose.orientation.x = q[0]
               goal_client.target_pose.pose.orientation.y = q[1]
               goal_client.target_pose.pose.orientation.z = q[2]
               goal_client.target_pose.pose.orientation.w = q[3]
               goal_client.target_pose.pose.position.x =config[i][0]
               goal_client.target_pose.pose.position.y =config[i][1]          
               rospy.sleep(0.1)
               
               client.send_goal(goal_client)
               rospy.loginfo(f"Cycle {j} configuration {i+1}, going to point")
               state=client.wait_for_result()


               if state:
                    result_state=client.get_state()
                    if result_state == 4:
                         rospy.loginfo(f"Cycle{j} Configuration {i+1} aborted")
                         rospy.loginfo(f"Cycle{j} aborted")
                         client.cancel_goal()
                         
                    elif result_state == 3:
                         rospy.loginfo("Point reached")

                         #Transformation quaternion to euler
                         amcl_orientation=tf.transformations.euler_from_quaternion([amcl.orientation.x,
                                                                                amcl.orientation.y,
                                                                                amcl.orientation.z,
                                                                                amcl.orientation.w
                                                                                ])
                         ground_truth_orientation=tf.transformations.euler_from_quaternion([ground_truth.pose.orientation.x,
                                                                                ground_truth.pose.orientation.y,
                                                                                ground_truth.pose.orientation.z,
                                                                                ground_truth.pose.orientation.w
                                                                                ])
                    
                         row=TableRow()

                         cell=TableCell()
                         cell.addElement(P(text=str(j)))
                         row.addElement(cell)
                    
                         cell=TableCell()
                         cell.addElement(P(text=str(i+1)))
                         row.addElement(cell)
                    
                         cell=TableCell()
                         cell.addElement(P(text=str(amcl.position.x)))
                         row.addElement(cell)
                         
                         cell=TableCell()
                         cell.addElement(P(text=str(amcl.position.y)))
                         row.addElement(cell)
                         
                         cell=TableCell()
                         cell.addElement(P(text=str(amcl_orientation[2])))
                         row.addElement(cell)
                         
                         cell=TableCell()
                         cell.addElement(P(text=str(ground_truth.pose.position.x)))
                         row.addElement(cell)
                         
                         cell=TableCell()
                         cell.addElement(P(text=str(ground_truth.pose.position.y)))
                         row.addElement(cell)
                         
                         cell=TableCell()
                         cell.addElement(P(text=str(ground_truth_orientation[2])))
                         row.addElement(cell)

                         table.addElement(row)
                         
                    else:
                         rospy.loginfo("Unknown state: %d", result_state)

    doc.spreadsheet.addElement(table)
    doc.save("positions.ods")