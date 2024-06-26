#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import RPi.GPIO as GPIO
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time

# Inicializar nodo ROS
rospy.init_node('advance_2m_with_aruco_detection', anonymous=True)

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
        self.stop_detected = False  # Variable para detener el movimiento si se detecta un ArUco

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

                # Si se detecta el ArUco con los datos especificados, detener el movimiento
                if aruco_size >= 35000.0 or 0.020 <= pose_msg.pose.position.x <= 0.025:
                    self.stop_detected = True
                    rospy.loginfo(f"Detención por ArUco detectado con tamaño {aruco_size} o posición x en el rango [0.020, 0.023] ; {pose_msg.pose.position.x}")

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

# Función para mover los motores en línea recta con detección de ArUco
def move_straight_2m():
    global start_time, end_time, encoder_left_count, encoder_right_count
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(85)
    pwm_right.ChangeDutyCycle(100)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    distancia_cm = 200
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        if aruco_detector.stop_detected:
            rospy.loginfo("Detenido por detección de ArUco")
            break
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado o detenido por ArUco: Motores detenidos")

    # Publicar los resultados finales
    rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Time elapsed: {end_time - start_time:.2f} seconds")

# Función para manejar el comando A1.1
def handle_A1_1():
    move_straight_2m()
    done_pub.publish("Done")
    rospy.loginfo("Mensaje 'Done' publicado")

def reset_state():
    global encoder_left_count, encoder_right_count
    encoder_left_count = 0
    encoder_right_count = 0
    aruco_detector.stop_detected = False
    aruco_detector.detected_aruco_data = []

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    reset_state()
    if data.data == "A1.1":
        handle_A1_1()

# Publicador para el mensaje "finish_move"
done_pub = rospy.Publisher('finish_move', String, queue_size=10)

# Nodo para escuchar comandos
rospy.Subscriber('first_move', String, command_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
        rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
        rospy.loginfo(f"Time elapsed: {end_time - start_time:.2f} seconds")
