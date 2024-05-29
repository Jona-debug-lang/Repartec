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

# Función para ajustar el movimiento basado en la desviación del Lidar hacia adelante en eje Y
def adjust_movement_forward_y(deviation):
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

# Función para ajustar el movimiento basado en la desviación del Lidar hacia adelante en eje X
def adjust_movement_forward_x(deviation):
    if deviation > 0.08:  # Desviación positiva mayor a 8 cm
        pwm_left.ChangeDutyCycle(97)
        pwm_right.ChangeDutyCycle(87)
    elif deviation < -0.08:  # Desviación negativa mayor a 8 cm
        pwm_left.ChangeDutyCycle(97)
        pwm_right.ChangeDutyCycle(65)
    else:
        pwm_left.ChangeDutyCycle(97)
        pwm_right.ChangeDutyCycle(87)
    rospy.loginfo(f"Adjusting due to X deviation (forward): {deviation:.3f}")

# Función para ajustar el movimiento basado en la desviación del Lidar hacia atrás en eje Y
def adjust_movement_backward_y(deviation):
    if deviation > 0.08:  # Desviación positiva mayor a 8 cm
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(95)
    elif deviation < -0.08:  # Desviación negativa mayor a 8 cm
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(80)
    else:
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(95)
    rospy.loginfo(f"Adjusting due to Y deviation (backward): {deviation:.3f}")

# Función para ajustar el movimiento basado en la desviación del Lidar hacia atrás en eje X
def adjust_movement_backward_x(deviation):
    if deviation > 0.08:  # Desviación positiva mayor a 8 cm
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(95)
    elif deviation < -0.08:  # Desviación negativa mayor a 8 cm
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(80)
    else:
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(95)
    rospy.loginfo(f"Adjusting due to X deviation (backward): {deviation:.3f}")

# Función para mover los motores en línea recta
def move_straight(distancia_cm, pwm_left_speed, pwm_right_speed, eje_correccion, direccion="forward"):
    global encoder_left_count, encoder_right_count, should_stop, initial_x, initial_y
    
    if direccion == "forward":
        GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
        GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
        GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    else:
        GPIO.output(MOTOR_IZQ_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IZQ_IN2, GPIO.HIGH)
        GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
        GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)

    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    if eje_correccion == 'y':
        initial_x = current_x
        objetivo = initial_x - (distancia_cm / 100.0)  # Distancia negativa para avanzar
    elif eje_correccion == 'x':
        initial_y = deviation_y
        objetivo = initial_y + (distancia_cm / 100.0)  # Distancia positiva para avanzar
    
    rospy.loginfo(f"Inicio del movimiento recto de {distancia_cm} cm")
    
    while (encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der):
        if eje_correccion == 'y':
            if direccion == "forward":
                adjust_movement_forward_y(deviation_y)
                if current_x <= objetivo:
                    break
            else:
                adjust_movement_backward_y(deviation_y)
                if current_x >= objetivo:
                    break
        elif eje_correccion == 'x':
            if direccion == "forward":
                adjust_movement_forward_x(current_x)
                if abs(deviation_y) >= abs(objetivo):
                    break
            else:
                adjust_movement_backward_x(current_x)
                if abs(deviation_y) >= abs(objetivo):
                    break
        if encoder_left_count >= pulsos_deseados_izq or encoder_right_count >= pulsos_deseados_der:
            break
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
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)
            rospy.loginfo("Movimiento reanudado")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado: Motores detenidos")

# Función para manejar el comando T1
def handle_T1():
    move_straight(200, 97, 87, 'y', 'forward')
    done_pub.publish("Forward Done")
    rospy.loginfo("Mensaje 'Forward Done' publicado")
    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    move_straight(200, 96, 95, 'y', 'backward')
    done_pub.publish("Backward Done")
    rospy.loginfo("Mensaje 'Backward Done' publicado")

# Función para manejar el comando T2
def handle_T2():
    # Primera parte de la trayectoria T2
    move_straight(70, 85, 100, 'y')
    turn_90_degrees(5000, 100, 0)
    move_straight(40, 85, 100, 'x')
    turn_90_degrees(6750, 100, 0)
    move_straight(45, 85, 100, 'y')
    
    done_pub.publish("T2 Part 1 Done")
    rospy.loginfo("Mensaje 'T2 Part 1 Done' publicado")
    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    
    # Segunda parte de la trayectoria T2
    move_straight(60, 70, 80, 'y')
    turn_in_place_90_degrees(800, 50)
    move_straight(200, 55, 75, 'x')
    turn_in_place_90_degrees(800, 50)
    move_straight(60, 70, 80, 'y')
    
    done_pub.publish("T2 Done")
    rospy.loginfo("Mensaje 'T2 Done' publicado")

# Función para giro de 90 grados
def turn_90_degrees(duration_ms, pwm_left_speed, pwm_right_speed):
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    time.sleep(duration_ms / 1000.0)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

# Función para giro en su propio eje de 90 grados
def turn_in_place_90_degrees(duration_ms, pwm_speed):
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(pwm_speed)
    pwm_right.ChangeDutyCycle(pwm_speed)
    time.sleep(duration_ms / 1000.0)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def reset_state():
    global encoder_left_count, encoder_right_count, deviation_y, current_x
    encoder_left_count = 0
    encoder_right_count = 0
    deviation_y = 0
    current_x = 0

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    reset_state()
    if data.data == "T1":
        handle_T1()
    elif data.data == "T2":
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
ubuntu@ubuntu:~$ rosrun my_python_scripts T2.lidar.py
[INFO] [1716963463.496193]: Received Lidar data: x=-0.004, y=-0.005, theta=-0.00
[INFO] [1716963463.605346]: Received Lidar data: x=-0.002, y=-0.004, theta=0.00
[INFO] [1716963463.712999]: Received Lidar data: x=0.002, y=-0.003, theta=0.00
[INFO] [1716963463.820164]: Received Lidar data: x=-0.004, y=-0.006, theta=-0.00
[INFO] [1716963463.923850]: Received Lidar data: x=-0.003, y=-0.007, theta=-0.00
[INFO] [1716963464.014778]: Received Lidar data: x=0.000, y=-0.004, theta=0.00
[INFO] [1716963464.120180]: Received Lidar data: x=-0.003, y=-0.007, theta=-0.00
[INFO] [1716963464.235595]: Received Lidar data: x=-0.001, y=-0.007, theta=0.00
[INFO] [1716963464.338672]: Received Lidar data: x=0.001, y=-0.005, theta=0.00
[INFO] [1716963464.447306]: Received Lidar data: x=-0.004, y=-0.006, theta=-0.00
[INFO] [1716963464.558933]: Received Lidar data: x=-0.005, y=-0.007, theta=-0.00
[INFO] [1716963464.666706]: Received Lidar data: x=-0.003, y=-0.006, theta=-0.00
[INFO] [1716963464.773288]: Received Lidar data: x=-0.007, y=-0.008, theta=-0.01
[INFO] [1716963464.924611]: Received Lidar data: x=-0.007, y=-0.009, theta=-0.00
[INFO] [1716963465.033806]: Received Lidar data: x=-0.006, y=-0.010, theta=-0.00
[INFO] [1716963465.138336]: Received Lidar data: x=-0.006, y=-0.009, theta=-0.00
[INFO] [1716963465.236045]: Comando recibido: T2
[INFO] [1716963465.245580]: Inicio del movimiento recto de 70 cm
[INFO] [1716963465.252247]: Received Lidar data: x=-0.006, y=-0.010, theta=-0.01
[INFO] [1716963465.258616]: Adjusting due to Y deviation (forward): -0.010
[INFO] [1716963465.355818]: Received Lidar data: x=-0.007, y=-0.009, theta=-0.00
[INFO] [1716963465.461009]: Received Lidar data: x=-0.002, y=-0.006, theta=0.00
[INFO] [1716963465.567296]: Received Lidar data: x=-0.005, y=-0.007, theta=-0.00
[INFO] [1716963465.670368]: Received Lidar data: x=-0.001, y=-0.004, theta=0.00
[INFO] [1716963465.776694]: Received Lidar data: x=-0.004, y=-0.008, theta=-0.01
[INFO] [1716963465.884445]: Received Lidar data: x=-0.017, y=-0.011, theta=0.01
[INFO] [1716963465.994362]: Received Lidar data: x=-0.032, y=-0.013, theta=0.00
[INFO] [1716963466.103605]: Received Lidar data: x=-0.049, y=-0.014, theta=0.00
[INFO] [1716963466.211955]: Received Lidar data: x=-0.080, y=-0.018, theta=0.01
[INFO] [1716963466.320295]: Received Lidar data: x=-0.109, y=-0.021, theta=0.01
[INFO] [1716963466.427278]: Received Lidar data: x=-0.136, y=-0.022, theta=0.01
[INFO] [1716963466.550856]: Received Lidar data: x=-0.170, y=-0.024, theta=0.01
[INFO] [1716963466.673233]: Received Lidar data: x=-0.200, y=-0.029, theta=0.01
[INFO] [1716963466.796305]: Received Lidar data: x=-0.237, y=-0.034, theta=0.02
[INFO] [1716963466.916197]: Received Lidar data: x=-0.270, y=-0.038, theta=0.02
[INFO] [1716963467.038120]: Received Lidar data: x=-0.310, y=-0.043, theta=0.02
[INFO] [1716963467.157901]: Received Lidar data: x=-0.343, y=-0.047, theta=0.02
[INFO] [1716963467.298248]: Received Lidar data: x=-0.379, y=-0.054, theta=0.02
[INFO] [1716963467.419713]: Received Lidar data: x=-0.408, y=-0.058, theta=0.02
[INFO] [1716963467.539472]: Received Lidar data: x=-0.444, y=-0.064, theta=0.03
[INFO] [1716963467.663863]: Received Lidar data: x=-0.487, y=-0.070, theta=0.03
[INFO] [1716963467.786628]: Received Lidar data: x=-0.524, y=-0.076, theta=0.03
[INFO] [1716963467.916971]: Received Lidar data: x=-0.565, y=-0.082, theta=0.04
[INFO] [1716963468.036006]: Received Lidar data: x=-0.594, y=-0.086, theta=0.04
[INFO] [1716963468.157017]: Received Lidar data: x=-0.634, y=-0.093, theta=0.05
[INFO] [1716963468.282853]: Received Lidar data: x=-0.667, y=-0.098, theta=0.04
[INFO] [1716963468.395678]: Received Lidar data: x=-0.705, y=-0.103, theta=0.05
[INFO] [1716963468.505394]: Received Lidar data: x=-0.733, y=-0.108, theta=0.05
[INFO] [1716963468.619185]: Received Lidar data: x=-0.769, y=-0.114, theta=0.05
[INFO] [1716963468.749143]: Received Lidar data: x=-0.801, y=-0.119, theta=0.05
[INFO] [1716963468.861985]: Received Lidar data: x=-0.841, y=-0.126, theta=0.05
[INFO] [1716963468.972022]: Received Lidar data: x=-0.871, y=-0.131, theta=0.06
[INFO] [1716963469.083382]: Received Lidar data: x=-0.898, y=-0.136, theta=0.06
[INFO] [1716963469.177836]: Received Lidar data: x=-0.936, y=-0.143, theta=0.06
[INFO] [1716963469.289221]: Received Lidar data: x=-0.966, y=-0.150, theta=0.06
[INFO] [1716963469.400394]: Received Lidar data: x=-0.992, y=-0.154, theta=0.07
[INFO] [1716963469.511590]: Received Lidar data: x=-1.016, y=-0.159, theta=0.07
[INFO] [1716963469.621053]: Received Lidar data: x=-1.043, y=-0.163, theta=0.07
[INFO] [1716963469.730237]: Received Lidar data: x=-1.065, y=-0.167, theta=0.07
^C[INFO] [1716963473.730724]: GPIO cleanup and node shutdown.
