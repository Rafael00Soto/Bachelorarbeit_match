#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import copy


def map_callback(msg):
    
    resolution=msg.info.resolution

    
    # Copia profunda del mensaje del OccupancyGrid para modificarlo
    updated_map = copy.deepcopy(msg)

    updated_map.header.stamp = rospy.Time.now()
    updated_map.header.frame_id = msg.header.frame_id 
    # Longitudes en metros del objeto
    length_x = 1.0
    length_y = 1.0

    # Obtener la información necesaria del OccupancyGrid
    width = updated_map.info.width
    height = updated_map.info.height
    origin_x = updated_map.info.origin.position.x
    origin_y = updated_map.info.origin.position.y

    # Coordenadas del centro del objeto en metros
    x_center = 0.0
    y_center = 3.0

    # Convertir longitudes de metros a número de celdas en el mapa
    cells_x = int(length_x / resolution)
    cells_y = int(length_y / resolution)

    # Convertir coordenadas del centro del objeto a índices de celda en el mapa
    x_index_center = int((x_center - origin_x) / resolution)
    y_index_center = int((y_center - origin_y) / resolution)

    # Calcular el rango de celdas del objeto basado en su tamaño
    half_cells_x = int(cells_x / 2)
    half_cells_y = int(cells_y / 2)

    # Verificar que las coordenadas estén dentro de los límites del mapa
    if (x_index_center - half_cells_x >= 0 and x_index_center + half_cells_x < width and
            y_index_center - half_cells_y >= 0 and y_index_center + half_cells_y < height):
        # Convertir updated_map.data a una lista mutable para modificarla
        updated_data_list = list(updated_map.data)

        # Iterar sobre las celdas del objeto y marcarlas como ocupadas (100)
        for i in range(-half_cells_x, half_cells_x + 1):
            for j in range(-half_cells_y, half_cells_y + 1):
                index = (y_index_center + j) * width + (x_index_center + i)
                updated_data_list[index] = 100

        # Asignar la lista modificada de vuelta a updated_map.data
        updated_map.data = tuple(updated_data_list)

        # Publicar el OccupancyGrid actualizado en el topic /map
        map_publisher.publish(updated_map)
        rospy.loginfo(f"Modificado el OccupancyGrid")
        
    else:
        rospy.logwarn("Las coordenadas del objeto están fuera de los límites del mapa.")                     
    '''
    new_map=OccupancyGridUpdate()
    new_map.header.stamp = rospy.Time.now()
    new_map.header.frame_id = msg.header.frame_id 
    new_map.x=0
    new_map.y=0
    width_in_meters=3.0 # x direction
    height_in_meters=3.0# y direction
    #meters to number of cells
    new_map.width= int(width_in_meters / resolution)
    new_map.height= int(height_in_meters / resolution)
    new_map.data=[100]*(new_map.width*new_map.height)
    map_publisher.publish(new_map)
    rospy.sleep(1.0)
    
    rospy.loginfo(f"Modificado el OccupancyGrid en ({new_map.x}, {new_map.y}) con un objeto de tamaño {width_in_meters}x{height_in_meters}")   '''
    


    


if __name__ == '__main__':

    rospy.init_node('map_updater', anonymous=True)

    # Suscribirse al topic /map
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Publicar en el mismo topic /map para actualizar el mapa
    map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    

    rospy.spin()
