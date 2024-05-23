#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from obstacle_detector.msg import Obstacles
import RPi.GPIO as GPIO
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time

# Inicializar nodo ROS
rospy.init_node('straight_movement_with_aruco_detection', anonymous=True)

# Configuración de los GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definición de pines
MOTOR_IZQ_ENABLE = 12
MOTOR_IZQ_IN1 = 23
MOTOR_IZQ_IN2 = 24
MOTOR_DER_ENABLE = 13
MOTOR_DER_IN3 = 27
MOTOR_DER_IN4 = 22
ENCODER_IZQ = 5
ENCODER_DER = 6

# Configurar pines de motor y encoder
GPIO.setup([MOTOR_IZQ_ENABLE, MOTOR_DER_ENABLE, MOTOR_IZQ_IN1, MOTOR_IZQ_IN2, MOTOR_DER_IN3, MOTOR_DER_IN4], GPIO.OUT)
GPIO.setup([ENCODER_IZQ, ENCODER_DER], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Configurar PWM
pwm_left = GPIO.PWM(MOTOR_IZQ_ENABLE, 1000)
pwm_right = GPIO.PWM(MOTOR_DER_ENABLE, 1000)
pwm_left.start(0)
pwm_right.start(0)

# Variables para el seguimiento de encoders
encoder_left_count = 0
encoder_right_count = 0

# Constantes k basadas en tus datos (pulsos por centímetro)
k_izq = 29.23
k_der = 29.57

# Clase para detección de ArUco
class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.pose_pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
        
        # Parámetros de calibración de la cámara (Reemplaza estos valores con los de tu calibración)
        self.camera_matrix = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])  # Ejemplo de matriz de cámara
        self.dist_coeffs = np.array([0.1, -0.25, 0, 0, 0.1])  # Ejemplo de coeficientes de distorsión
        
        self.marker_size = 0.18  # Tamaño del marcador en metros
        self.detected_aruco_data = []

    def callback(self, data):
        try:
            self.cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_image()
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_image(self):
        gray = cv2.cvtColor(self.cv1_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                aruco_id = ids[i][0]
                aruco_size = cv2.contourArea(corners[i])
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                rospy.loginfo(f"Detected ArUco ID: {aruco_id} with size: {aruco_size}")
                self.detected_aruco_data.append({
                    'id': aruco_id,
                    'size': aruco_size,
                    'position': tvec,
                    'quaternion': self.rotation_matrix_to_quaternion(cv2.Rodrigues(rvec)[0])
                })
                
                # Publicar la pose del marcador
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera"
                pose_msg.pose.position.x = tvec[0]
                pose_msg.pose.position.y = tvec[1]
                pose_msg.pose.position.z = tvec[2]
                pose_msg.pose.orientation = self.rotation_matrix_to_quaternion(cv2.Rodrigues(rvec)[0])
                
                self.pose_pub.publish(pose_msg)
                rospy.loginfo(f"Published ArUco ID: {aruco_id} pose with x: {tvec[0]}, y: {tvec[1]}, z: {tvec[2]}, quaternion: {pose_msg.pose.orientation}")

        # Dibujar los marcadores detectados
        self.cv1_image = aruco.drawDetectedMarkers(self.cv1_image, corners, ids)
        cv2.imshow("Camera View", self.cv1_image)
        cv2.waitKey(1)
    
    def rotation_matrix_to_quaternion(self, R):
        # Convertir matriz de rotación a cuaternión
        Q = np.zeros((4,))
        Q[0] = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        Q[1] = (R[2, 1] - R[1, 2]) / (4.0 * Q[0])
        Q[2] = (R[0, 2] - R[2, 0]) / (4.0 * Q[0])
        Q[3] = (R[1, 0] - R[0, 1]) / (4.0 * Q[0])
        return Q

# Instancia del detector de ArUco
aruco_detector = ArucoDetector()

# Función de callback para los encoders
def encoder_callback_izq(channel):
    global encoder_left_count
    encoder_left_count += 1

def encoder_callback_der(channel):
    global encoder_right_count
    encoder_right_count += 1

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_IZQ, GPIO.RISING, callback=encoder_callback_izq)
GPIO.add_event_detect(ENCODER_DER, GPIO.RISING, callback=encoder_callback_der)

# Función para mover los motores en línea recta
def move_straight_30cm(pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    
    distancia_cm = 22
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado: Motores detenidos")
    
    # Publicar los datos de ArUco detectados
    for data in aruco_detector.detected_aruco_data:
        rospy.loginfo(f"ArUco ID: {data['id']}, tamaño: {data['size']}, posición: {data['position']}, quaternion: {data['quaternion']}")

if __name__ == '__main__':
    try:
        rospy.loginfo("Ejecutando movimiento recto con detección de ArUco")
        move_straight_30cm(80, 62)  # Llamar a la función con las velocidades deseadas
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")



ubuntu@ubuntu:~/catkin_ws$ rosrun my_python_scripts full3.py
[INFO] [1716490196.071467]: Ejecutando movimiento recto con detección de ArUco
[INFO] [1716490196.078089]: Inicio del movimiento recto: Pulsos deseados izq: 643.0600000000001, der: 650.54
[INFO] [1716490196.083335]: Pulsos actuales: izq: 0, der: 0
[INFO] [1716490196.102876]: Pulsos actuales: izq: 5, der: 4
[INFO] [1716490196.122471]: Pulsos actuales: izq: 12, der: 11
[INFO] [1716490196.147551]: Pulsos actuales: izq: 20, der: 23
[INFO] [1716490196.158662]: Detected ArUco ID: 2 with size: 29816.0
[INFO] [1716490196.165166]: Published ArUco ID: 2 pose with x: 0.010743219499180204, y: 0.0350144908512947, z: 0.5077719657368963, quaternion: [0.23045437 0.97245954 0.03476659 0.00212375]
[INFO] [1716490196.168180]: Pulsos actuales: izq: 27, der: 30
[INFO] [1716490196.203543]: Pulsos actuales: izq: 41, der: 40
[INFO] [1716490196.233499]: Pulsos actuales: izq: 50, der: 52
[INFO] [1716490196.274875]: Pulsos actuales: izq: 67, der: 69
[INFO] [1716490196.305360]: Pulsos actuales: izq: 81, der: 83
[INFO] [1716490196.335115]: Pulsos actuales: izq: 96, der: 99
[INFO] [1716490196.358794]: Pulsos actuales: izq: 109, der: 113
[INFO] [1716490196.381879]: Pulsos actuales: izq: 121, der: 126
[INFO] [1716490196.406008]: Pulsos actuales: izq: 133, der: 138
[INFO] [1716490196.456649]: Pulsos actuales: izq: 156, der: 164
[INFO] [1716490196.480886]: Pulsos actuales: izq: 169, der: 177
[INFO] [1716490196.505021]: Pulsos actuales: izq: 180, der: 188
[INFO] [1716490196.540392]: Pulsos actuales: izq: 191, der: 203
[INFO] [1716490196.577712]: Pulsos actuales: izq: 199, der: 210
[INFO] [1716490196.599285]: Pulsos actuales: izq: 210, der: 221
[INFO] [1716490196.624012]: Pulsos actuales: izq: 218, der: 231
[INFO] [1716490196.672336]: Pulsos actuales: izq: 244, der: 256
[INFO] [1716490196.695148]: Pulsos actuales: izq: 256, der: 269
[INFO] [1716490196.718755]: Pulsos actuales: izq: 266, der: 280
[INFO] [1716490196.769380]: Pulsos actuales: izq: 294, der: 309
[INFO] [1716490196.802485]: Pulsos actuales: izq: 306, der: 319
[INFO] [1716490196.831545]: Pulsos actuales: izq: 324, der: 337
[INFO] [1716490196.875391]: Pulsos actuales: izq: 345, der: 362
[INFO] [1716490196.903668]: Pulsos actuales: izq: 360, der: 378
[INFO] [1716490196.945171]: Pulsos actuales: izq: 376, der: 393
[INFO] [1716490196.984366]: Pulsos actuales: izq: 402, der: 420
[INFO] [1716490197.017095]: Pulsos actuales: izq: 418, der: 437
[INFO] [1716490197.052834]: Pulsos actuales: izq: 438, der: 457
[INFO] [1716490197.094861]: Pulsos actuales: izq: 459, der: 477
[INFO] [1716490197.119846]: Pulsos actuales: izq: 471, der: 489
[INFO] [1716490197.145073]: Pulsos actuales: izq: 484, der: 503
[INFO] [1716490197.191127]: Pulsos actuales: izq: 504, der: 525
[INFO] [1716490197.221626]: Pulsos actuales: izq: 518, der: 540
[INFO] [1716490197.252572]: Pulsos actuales: izq: 535, der: 558
[INFO] [1716490197.294976]: Pulsos actuales: izq: 556, der: 582
[INFO] [1716490197.332363]: Pulsos actuales: izq: 576, der: 603
[INFO] [1716490197.375207]: Pulsos actuales: izq: 592, der: 618
[INFO] [1716490197.414160]: Pulsos actuales: izq: 607, der: 633
[INFO] [1716490197.455887]: Pulsos actuales: izq: 623, der: 647
[INFO] [1716490197.511480]: Movimiento completado: Motores detenidos
[INFO] [1716490197.538746]: Detected ArUco ID: 2 with size: 29816.0
[INFO] [1716490197.550042]: ArUco ID: 2, tamaño: 29816.0, posición: [0.01074322 0.03501449 0.50777197], quaternion: [0.23045437 0.97245954 0.03476659 0.00212375]
[INFO] [1716490197.564610]: Published ArUco ID: 2 pose with x: 0.010743219499180204, y: 0.0350144908512947, z: 0.5077719657368963, quaternion: [0.23045437 0.97245954 0.03476659 0.00212375]
[INFO] [1716490197.583178]: ArUco ID: 2, tamaño: 29816.0, posición: [0.01074322 0.03501449 0.50777197], quaternion: [0.23045437 0.97245954 0.03476659 0.00212375]
[INFO] [1716490197.615813]: GPIO cleanup and node shutdown.
[INFO] [1716490197.701305]: Detected ArUco ID: 2 with size: 30176.5
[INFO] [1716490197.717051]: Published ArUco ID: 2 pose with x: 0.008555709557462422, y: 0.03570486846913558, z: 0.5038438423506415, quaternion: [ 0.23760678  0.97062718  0.03627    -0.01050654]
ubuntu@ubuntu:~/catkin_ws$ 



