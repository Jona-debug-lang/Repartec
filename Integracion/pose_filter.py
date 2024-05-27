#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
import tf.transformations

class PoseFilter:
    def __init__(self):
        rospy.init_node('pose_filter', anonymous=True)
        
        self.pub = rospy.Publisher('simple_pose', Pose2D, queue_size=10)
        self.sub = rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, self.callback)
        
        self.last_x = None
        self.last_y = None
        self.alpha = 0.5  # Factor de suavizado para filtrado simple

    def callback(self, data):
        try:
            pose2d = Pose2D()
            pose2d.x = round(self.simple_filter(data.pose.pose.position.x, self.last_x), 3)
            pose2d.y = round(self.simple_filter(data.pose.pose.position.y, self.last_y), 3)
            
            # Conversión de quaternion a ángulo Euler para theta
            orientation = data.pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            pose2d.theta = round(euler[2], 2)  # Ángulo alrededor del eje Z
            
            self.pub.publish(pose2d)
            
            rospy.logdebug("Publicado pose simplificado: %s", pose2d)
            
            # Actualizar los últimos valores
            self.last_x = pose2d.x
            self.last_y = pose2d.y
        except Exception as e:
            rospy.logerr("Error en el procesamiento de datos de pose: %s", e)

    def simple_filter(self, new_value, last_value):
        if last_value is None:
            return new_value
        return self.alpha * new_value + (1 - self.alpha) * last_value

if __name__ == '__main__':
    pf = PoseFilter()
    try:
        rospy.loginfo("Iniciando nodo pose_filter")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo pose_filter detenido.")
