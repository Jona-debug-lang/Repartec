#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
import json

# Inicializar nodo ROS
rospy.init_node('obstacle_filter', anonymous=True, log_level=rospy.DEBUG)

# Publicador para los datos filtrados
filtered_data_pub = rospy.Publisher('/filtered_obstacles', String, queue_size=10)

# Función de callback para la detección de obstáculos
def obstacles_callback(data):
    filtered_data = []
    for circle in data.circles:
        filtered_entry = {
            'center_x': circle.center.x,
            'true_radius': circle.true_radius
        }
        filtered_data.append(filtered_entry)
    
    filtered_data_json = json.dumps(filtered_data)
    filtered_data_pub.publish(filtered_data_json)
    rospy.logdebug(f"Filtered Data: {filtered_data_json}")

# Suscribirse al tópico de obstáculos
rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

if __name__ == '__main__':
    try:
        rospy.loginfo("Iniciando nodo de filtrado de obstáculos")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        rospy.loginfo("Node shutdown.")
