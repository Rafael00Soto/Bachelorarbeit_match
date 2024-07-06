#! /usr/bin/env python
import rospy
import actionlib
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf.transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from odf.opendocument import OpenDocumentSpreadsheet
from odf.table import Table, TableRow, TableCell
from odf.text import P




def callback_ground_truth(pose_ground_truth):
     global ground_truth
     ground_truth= pose_ground_truth

def callback_amcl(pose_amcl):
     global amcl
     amcl=pose_amcl


if __name__=="__main__":
    
    rospy.init_node("define_goal_node", anonymous=True)
    rospy.sleep(0.1)
    client = actionlib.SimpleActionClient('/mur620/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()
    goal_client=MoveBaseGoal()

    rospy.sleep(1.0)
    listener1=rospy.Subscriber("/mur620/amcl_pose",PoseWithCovarianceStamped, callback_amcl)
    listener2=rospy.Subscriber("/mur620/ground_truth",Odometry, callback_ground_truth)

    #Desired positions
    config=[]
    config.append([5,-3, 0])  
    config.append([-6,-5, 20])
    config.append([9,-8, 40])
    config.append([4,3,60])
    config.append([3,-9,80])

    doc = OpenDocumentSpreadsheet()
    table = Table(name="Hoja1")
    
    columns = ["Cycle","Configuration number","Amcl Pose x", "Amcl Pose y", "Amcl Pose Orientation", "Ground Truth x", "Ground Truth y", "Ground Truth Orientation"]
    header_row = TableRow()
    for column in columns:
        cell = TableCell()
        cell.addElement(P(text=str(column)))
        header_row.addElement(cell)
    table.addElement(header_row)
    
    goal = PoseStamped()
    for j in range(1,31):
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
               client.wait_for_result()


               state=client.get_state()
               n=0
               while state !=  3:
                    if state == 4 and n==0:
                         rospy.loginfo(f"Cycle{j} Configuration {i+1} aborted")
                         n=1
                         break

                    if rospy.is_shutdown():
                         break
                    else:
                         pass

               if n==1:
                    rospy.loginfo(f"Cycle{j} Configuration {i+1} aborted")
                    table.addElement(row)
               elif n==0:

                    rospy.loginfo(f"Point reached")

                    #Transformation quaternion to euler
                    amcl_orientation=tf.transformations.euler_from_quaternion([amcl.pose.pose.orientation.x,
                                                                           amcl.pose.pose.orientation.y,
                                                                           amcl.pose.pose.orientation.z,
                                                                           amcl.pose.pose.orientation.w
                                                                           ])
                    ground_truth_orientation=tf.transformations.euler_from_quaternion([ground_truth.pose.pose.orientation.x,
                                                                           ground_truth.pose.pose.orientation.y,
                                                                           ground_truth.pose.pose.orientation.z,
                                                                           ground_truth.pose.pose.orientation.w
                                                                           ])
               
                    row=TableRow()

                    cell=TableCell()
                    cell.addElement(P(text=str(j)))
                    row.addElement(cell)
               
                    cell=TableCell()
                    cell.addElement(P(text=str(i+1)))
                    row.addElement(cell)
               
                    cell=TableCell()
                    cell.addElement(P(text=str(amcl.pose.pose.position.x)))
                    row.addElement(cell)
                    cell=TableCell()
                    cell.addElement(P(text=str(amcl.pose.pose.position.y)))
                    row.addElement(cell)
                    cell=TableCell()
                    cell.addElement(P(text=str(amcl_orientation[2])))
                    row.addElement(cell)
                    cell=TableCell()
                    cell.addElement(P(text=str(ground_truth.pose.pose.position.x)))
                    row.addElement(cell)
                    cell=TableCell()
                    cell.addElement(P(text=str(ground_truth.pose.pose.position.y)))
                    row.addElement(cell)
                    cell=TableCell()
                    cell.addElement(P(text=str(ground_truth_orientation[2])))
                    row.addElement(cell)

                    table.addElement(row)
    table2 = Table(name="Hoja2")
    header_row_table2= TableRow()


    columns_table2 = ["Target Value x", "Target Value y", "Target Value Orientation"]
    for column in columns_table2:
        cell = TableCell()
        cell.addElement(P(text=str(column)))
        header_row_table2.addElement(cell)
    table2.addElement(header_row_table2)
    for point in config:
         row=TableRow()
         cell=TableCell()
         cell.addElement(P(text=str(point[0])))
         row.addElement(cell)
         cell=TableCell()
         cell.addElement(P(text=str(point[1])))
         row.addElement(cell)
         cell=TableCell()
         cell.addElement(P(text=str(point[2])))
         row.addElement(cell)
    table2.addElement(row)
     
            

    doc.spreadsheet.addElement(table)
    doc.save("mi_hoja_de_calculo.ods")