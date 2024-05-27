#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

def publish_coordinates():
    rospy.init_node('goal_publisher', anonymous=True)
    pub = rospy.Publisher('coordenadas_goal', Point, queue_size=10)

    # Crear el mensaje Point
    point = Point()
    point.x = 2  # Asignar 2 a x
    point.y = 0  # Asignar 0 a y
    point.z = 0  # Asignar 0 a z

    # Esperar a que los suscriptores estén listos
    rospy.sleep(1)
    rospy.loginfo("Publicando coordenadas al tópico 'coordenadas_goal'")
    pub.publish(point)
    
    # Publicar varias veces o esperar un poco para asegurar la recepción
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_coordinates()
    except rospy.ROSInterruptException:
        pass
