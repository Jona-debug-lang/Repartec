#!/usr/bin/env python3

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String
import json

# Inicializar nodo ROS
rospy.init_node('filter_obstacles', anonymous=True, log_level=rospy.DEBUG)

# Publicador para los datos filtrados de obstáculos
filtered_obstacles_pub = rospy.Publisher('/filtered_obstacles', String, queue_size=10)

# Función de callback para la detección de obstáculos
def obstacles_callback(data):
    filtered_obstacles = []
    for circle in data.circles:
        if not hasattr(circle, 'center') or not hasattr(circle.center, 'x') or not hasattr(circle, 'radius'):
            continue
        obstacle_info = {
            'center_x': abs(circle.center.x),  # Usar el eje X como distancia frontal
            'radius': circle.radius
        }
        filtered_obstacles.append(obstacle_info)
    filtered_obstacles_str = json.dumps(filtered_obstacles)
    rospy.logdebug(f"Datos filtrados: {filtered_obstacles_str}")
    filtered_obstacles_pub.publish(filtered_obstacles_str)

# Suscribirse al tópico de obstáculos
rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

# Establecer la frecuencia de procesamiento
rate = rospy.Rate(10)  # 10 Hz

if __name__ == '__main__':
    try:
        rospy.loginfo("Iniciando nodo de filtrado de obstáculos")
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
