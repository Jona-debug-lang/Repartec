#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose2D
from obstacle_detector.msg import Obstacles
import RPi.GPIO as GPIO
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time

# Inicializar nodo ROS
rospy.init_node('integrated_robot_movement', anonymous=True)

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

# Variables para la corrección de ruta y detección de distancia
deviation_y = 0
current_x = 0

# Constantes k basadas en tus datos (pulsos por centímetro)
k_izq = 29.23
k_der = 29.57

# Variables para control de detención
should_stop = False
MAX_SPEED = 0.25  # Velocidad máxima para detenerse
DETECTION_RADIUS = 2.5  # Radio de detección para detenerse

# Publicador para los datos de obstáculos
obstacle_data_pub = rospy.Publisher('obstacle_data', String, queue_size=10)

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

# Suscripción a la posición del Lidar
def pose_callback(data):
    global deviation_y, current_x
    deviation_y = data.y
    current_x = data.x
    rospy.loginfo("Received Lidar data: x={:.3f}, y={:.3f}, theta={:.2f}".format(data.x, data.y, data.theta))

rospy.Subscriber('simple_pose', Pose2D, pose_callback)

# Función para verificar obstáculos
def check_obstacles():
    global should_stop
    should_stop = False
    obstacles_msg = rospy.wait_for_message('/obstacles', Obstacles)
    rospy.loginfo("Datos de obstáculos recibidos")
    for circle in obstacles_msg.circles:
        rospy.loginfo(f"Obstáculo detectado: posición=({circle.center.x}, {circle.center.y}), velocidad=({circle.velocity.x}, {circle.velocity.y}), radio={circle.radius}")
        speed = (circle.velocity.x ** 2 + circle.velocity.y ** 2) ** 0.5
        if speed > MAX_SPEED:
            # Verificar si el obstáculo está fuera de las áreas seguras del robot
            if (circle.center.y > 0.20 or
                circle.center.y < -0.20 or
                circle.center.x > 0.32):
                should_stop = True
                rospy.loginfo(f"Detenerse: Objeto en movimiento detectado en x: {circle.center.x}, y: {circle.center.y}")
                return

# Función para ajustar el movimiento basado en la desviación del Lidar hacia adelante
def adjust_movement_forward(deviation):
    if deviation > 0.08:  # Desviación positiva mayor a 8 cm
        pwm_left.ChangeDutyCycle(97)  # Ajustar duty cycle para corrección
        pwm_right.ChangeDutyCycle(87)
    elif deviation < -0.08:  # Desviación negativa mayor a 8 cm
        pwm_left.ChangeDutyCycle(97)
        pwm_right.ChangeDutyCycle(65)                     
    else:
        # Restablecer los PWM a valores normales si la desviación es menor o igual a 8 cm
        pwm_left.ChangeDutyCycle(97)
        pwm_right.ChangeDutyCycle(87)
    rospy.loginfo(f"Adjusting due to Y deviation (forward): {deviation:.3f}")

# Función para mover los motores en línea recta hacia adelante con corrección
def move_straight_70cm_forward():
    global start_time, end_time, encoder_left_count, encoder_right_count, current_x, should_stop
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(99)
    pwm_right.ChangeDutyCycle(76)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = 70 * k_izq  # 70 cm * k_izq
    pulsos_deseados_der = 70 * k_der  # 70 cm * k_der
    
    rospy.loginfo("Inicio del movimiento recto de 70 cm hacia adelante")
    
    while (encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der) and current_x > -0.50:
        adjust_movement_forward(deviation_y)
        check_obstacles()  # Verificar obstáculos en cada iteración
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Motores detenidos temporalmente")
            time.sleep(5)  # Pausa por 5 segundos
            rospy.loginfo("Revisando nuevamente después de la pausa")
            for _ in range(20):  # Revisar durante 2 segundos (20 ciclos de 0.1 segundos)
                check_obstacles()
                if should_stop:
                    rospy.loginfo("El obstáculo aún está presente. Pausando nuevamente.")
                    time.sleep(5)
                else:
                    break
            pwm_left.ChangeDutyCycle(99)
            pwm_right.ChangeDutyCycle(87)
            rospy.loginfo("Movimiento reanudado")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")
    
    # Publicar los resultados finales
    rospy.loginfo(f"Forward counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Forward time elapsed: {end_time - start_time:.2f} seconds")

# Función para realizar una curva de 90 grados hacia la derecha
def turn_90_degrees(time_T):
    rospy.loginfo("Inicio de la curva de 90 grados")
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    
    pwm_left.ChangeDutyCycle(100)
    pwm_right.ChangeDutyCycle(25)
    
    time.sleep(time_T)  # Ajusta este tiempo según la calibración necesaria
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Curva de 90 grados completada")

# Función para ajustar el movimiento basado en la desviación del Lidar hacia adelante
def adjust_movement_forward_30cm(deviation):
    if deviation > 0.08:  # Desviación positiva mayor a 8 cm
        pwm_left.ChangeDutyCycle(97)  # Ajustar duty cycle para corrección
        pwm_right.ChangeDutyCycle(87)
    elif deviation < -0.08:  # Desviación negativa mayor a 8 cm
        pwm_left.ChangeDutyCycle(97)
        pwm_right.ChangeDutyCycle(65)                     
    else:
        # Restablecer los PWM a valores normales si la desviación es menor o igual a 8 cm
        pwm_left.ChangeDutyCycle(97)
        pwm_right.ChangeDutyCycle(87)
    rospy.loginfo(f"Adjusting due to Y deviation (forward): {deviation:.3f}")

# Función para mover los motores en línea recta 30 cm hacia adelante con corrección
def move_straight_30cm_forward():
    global start_time, end_time, encoder_left_count, encoder_right_count, current_x, should_stop, deviation_y
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(97)
    pwm_right.ChangeDutyCycle(87)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = 30 * k_izq  # 30 cm * k_izq
    pulsos_deseados_der = 30 * k_der  # 30 cm * k_der
    
    initial_deviation = deviation_y
    
    rospy.loginfo("Inicio del movimiento recto de 30 cm hacia adelante")
    
    while (encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der) and initial_deviation < -0.30:
        adjust_movement_forward_30cm(initial_deviation)
        check_obstacles()  # Verificar obstáculos en cada iteración
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Motores detenidos temporalmente")
            time.sleep(5)  # Pausa por 5 segundos
            rospy.loginfo("Revisando nuevamente después de la pausa")
            for _ in range(20):  # Revisar durante 2 segundos (20 ciclos de 0.1 segundos)
                check_obstacles()
                if should_stop:
                    rospy.loginfo("El obstáculo aún está presente. Pausando nuevamente.")
                    time.sleep(5)
                else:
                    break
            pwm_left.ChangeDutyCycle(97)
            pwm_right.ChangeDutyCycle(87)
            rospy.loginfo("Movimiento reanudado")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")
    
    # Publicar los resultados finales
    rospy.loginfo(f"Forward counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Forward time elapsed: {end_time - start_time:.2f} seconds")

def handle_T2():
    move_straight_70cm_forward()
    done_pub.publish("Forward Done")
    rospy.loginfo("Mensaje 'Forward Done' publicado")
    turn_90_degrees(8.5)
    done_pub.publish("Turn Done")
    rospy.loginfo("Mensaje 'Turn Done' publicado")
    move_straight_30cm_forward()
    done_pub.publish("Forward 30cm Done")
    rospy.loginfo("Mensaje 'Forward 30cm Done' publicado")
    turn_90_degrees(8.5)
    done_pub.publish("Final Turn Done")
    rospy.loginfo("Mensaje 'Final Turn Done' publicado")

def reset_state():
    global encoder_left_count, encoder_right_count, deviation_y, current_x
    encoder_left_count = 0
    encoder_right_count = 0
    deviation_y = 0
    current_x = 0

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    reset_state()
    if data.data == "T2":
        handle_T2()

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
