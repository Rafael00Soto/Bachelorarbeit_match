#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid



def lod_box_rviz():
  pass

def load_box_gazebo():

    # Esperar a que el servicio de SpawnModel esté disponible
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        # Crear un proxy para el servicio SpawnModel
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        # Definir el modelo de la caja en formato SDF
        box_sdf = """
        <sdf version="1.6">
          <model name="box">
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
              <pose>0 0 0.5 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>1 1 1</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>1 1 1</size>
                  </box>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        """

        # Definir la posición inicial de la caja
        initial_pose = Pose(Point(2, 3, 0.5), Quaternion(0, 0, 0, 0))

        # Llamar al servicio para insertar el modelo en la simulación
        spawn_model_prox("box", box_sdf, "", initial_pose, "world")

        rospy.loginfo("Caja insertada en Gazebo")

    except rospy.ServiceException as e:
        rospy.logerr("Servicio de inserción falló: %s" % e)

if __name__ == '__main__':
    rospy.init_node('load_box_node', anonymous=True)
    try:
        load_box_gazebo()
    except rospy.ROSInterruptException:
        pass