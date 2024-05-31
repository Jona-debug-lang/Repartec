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

# Publicador para el mensaje "finish_move"
boton = rospy.Publisher('boton', String, queue_size=10)

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
MAX_SPEED = 0.10  # Velocidad máxima para detenerse
DETECTION_RADIUS = 2.5  # Radio de detección para detenerse

# Publicador para los datos de obstáculos
obstacle_data_pub = rospy.Publisher('obstacle_data', String, queue_size=10)

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.pose_pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
        self.camera_matrix = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
        self.dist_coeffs = np.array([0.1, -0.25, 0, 0, 0.1])
        self.marker_size = 0.18

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

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera"
                pose_msg.pose.position.x = tvec[0]
                pose_msg.pose.position.y = tvec[1]
                pose_msg.pose.position.z = tvec[2]
                pose_msg.pose.orientation = self.rotation_matrix_to_quaternion(cv2.Rodrigues(rvec)[0])
                
                self.pose_pub.publish(pose_msg)
                rospy.loginfo(f"Published ArUco ID: {aruco_id} pose with x: {tvec[0]}, y: {tvec[1]}, z: {tvec[2]}, quaternion: {pose_msg.pose.orientation}")

        self.cv1_image = aruco.drawDetectedMarkers(self.cv1_image, corners, ids)
        #cv2.imshow("Camera View", self.cv1_image)
        #cv2.waitKey(1)
    
    def rotation_matrix_to_quaternion(self, R):
        Q = np.zeros((4,))
        Q[0] = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        Q[1] = (R[2, 1] - R[1, 2]) / (4.0 * Q[0])
        Q[2] = (R[0, 2] - R[2, 0]) / (4.0 * Q[0])
        Q[3] = (R[1, 0] - R[0, 1]) / (4.0 * Q[0])
        return Q

# Instancia del detector de ArUco
aruco_detector = ArucoDetector()

def encoder_callback_izq(channel):
    global encoder_left_count
    encoder_left_count += 1

def encoder_callback_der(channel):
    global encoder_right_count
    encoder_right_count += 1

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_IZQ, GPIO.RISING, callback=encoder_callback_izq)
GPIO.add_event_detect(ENCODER_DER, GPIO.RISING, callback=encoder_callback_der)

def pose_callback(data):
    global deviation_y, current_x
    deviation_y = data.y
    current_x = data.x
    rospy.loginfo("Received Lidar data: x={:.3f}, y={:.3f}, theta={:.2f}".format(data.x, data.y, data.theta))

rospy.Subscriber('simple_pose', Pose2D, pose_callback)

def check_obstacles():
    global should_stop
    should_stop = False
    obstacles_msg = rospy.wait_for_message('/obstacles', Obstacles)
    rospy.loginfo("Datos de obstáculos recibidos")
    for circle in obstacles_msg.circles:
        rospy.loginfo(f"Obstáculo detectado: posición=({circle.center.x}, {circle.center.y}), velocidad=({circle.velocity.x}, {circle.velocity.y}), radio={circle.radius}")
        speed_x = circle.velocity.x
        speed_y = circle.velocity.y
        if speed_x > MAX_SPEED or speed_y > MAX_SPEED:
            if (circle.center.y > 0.50 or circle.center.y < -0.50 or circle.center.x > 0.50 or circle.center.x < -0.50 or circle.radius > 0.15):
                should_stop = True
                rospy.loginfo(f"Detenerse: Objeto en movimiento detectado en x: {circle.center.x}, y: {circle.center.y}")
                return

def adjust_movement(deviation, duty_cycle_high=97, duty_cycle_low=65):
    if deviation > 0.08:
        pwm_left.ChangeDutyCycle(duty_cycle_high)
        pwm_right.ChangeDutyCycle(87)
    elif deviation < -0.08:
        pwm_left.ChangeDutyCycle(duty_cycle_high)
        pwm_right.ChangeDutyCycle(duty_cycle_low)
    else:
        pwm_left.ChangeDutyCycle(duty_cycle_high)
        pwm_right.ChangeDutyCycle(87)
    rospy.loginfo(f"Adjusting due to Y deviation (forward): {deviation:.3f}")

def move_straight(distance_cm, duty_cycle_high=97, duty_cycle_low=87):
    global encoder_left_count, encoder_right_count, current_x, should_stop
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(duty_cycle_high)
    pwm_right.ChangeDutyCycle(duty_cycle_low)
    
    encoder_left_count = 0
    encoder_right_count = 0
    
    pulsos_deseados_izq = distance_cm * k_izq
    pulsos_deseados_der = distance_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento recto de {distance_cm} cm hacia adelante")
    
    while (encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der):
        adjust_movement(deviation_y)
        check_obstacles()
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Motores detenidos temporalmente")
            time.sleep(5)
            rospy.loginfo("Revisando nuevamente después de la pausa")
            for _ in range(20):
                check_obstacles()
                if should_stop:
                    rospy.loginfo("El obstáculo aún está presente. Pausando nuevamente.")
                    time.sleep(5)
                else:
                    break
            pwm_left.ChangeDutyCycle(duty_cycle_high)
            pwm_right.ChangeDutyCycle(duty_cycle_low)
            rospy.loginfo("Movimiento reanudado")
        time.sleep(0.01)
    
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")
    rospy.loginfo(f"Forward counts: Left {encoder_left_count}, Right {encoder_right_count}")

def turn(degree, time_T, motor_left_duty=100, motor_right_duty=25, reverse=False):
    global should_stop
    rospy.loginfo(f"Inicio de la curva de {degree} grados")
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    
    if reverse:
        GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
        GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
        motor_left_duty = 85
        motor_right_duty = 85
    else:
        GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
        GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    
    pwm_left.ChangeDutyCycle(motor_left_duty)
    pwm_right.ChangeDutyCycle(motor_right_duty)
    
    elapsed_time = 0
    
    while elapsed_time < time_T:
        start_time = time.time()
        check_obstacles()
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Motores detenidos temporalmente")
            while should_stop:
                time.sleep(0.1)
                check_obstacles()
            rospy.loginfo("Movimiento reanudado")
            pwm_left.ChangeDutyCycle(motor_left_duty)
            pwm_right.ChangeDutyCycle(motor_right_duty)
        else:
            time.sleep(0.01)
            elapsed_time += time.time() - start_time
    
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo(f"Curva de {degree} grados completada")


def handle_T2():
    move_straight(50)
    done_pub.publish("Forward Done")
    rospy.loginfo("Mensaje 'Forward Done' publicado")
    turn(90, 8.5)
    done_pub.publish("Turn Done")
    rospy.loginfo("Mensaje 'Turn Done' publicado")
    move_straight(10)
    done_pub.publish("Forward 30cm Done")
    rospy.loginfo("Mensaje 'Forward 30cm Done' publicado")
    turn(90, 10)
    done_pub.publish("Final Turn Done")
    rospy.loginfo("Mensaje 'Final Turn Done' publicado")
    move_straight(40)
    done_pub.publish("Forward 20cm Done")
    rospy.loginfo("Mensaje 'Forward 20cm Done' publicado")
    
def handle_T21():
    move_straight(20, duty_cycle_high=77, duty_cycle_low=70)
    done_pub.publish("Forward 20cm Done")
    rospy.loginfo("Mensaje 'Forward 20cm Done' publicado")
    turn(90, 2.5, reverse=True)
    done_pub.publish("Final Turn Done")
    rospy.loginfo("Mensaje 'Final Turn Done' publicado")
    move_straight(200, duty_cycle_high=77, duty_cycle_low=67)
    done_pub.publish("Forward 200cm Done")
    rospy.loginfo("Mensaje 'Forward 0200cm Done' publicado")
    turn(90, 2.5, reverse=True)
    done_pub.publish("Final Turn Done")
    rospy.loginfo("Mensaje 'Final Turn Done' publicado")
    move_straight(25, duty_cycle_high=77, duty_cycle_low=70)
    done_pub.publish("Forward 20cm Done")
    rospy.loginfo("Mensaje 'Forward 20cm Done' publicado")

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
        boton.publish("GRACIAS")
    elif data.data == "continue":
        handle_T21()

# Publicador para el mensaje "finish_move"
done_pub = rospy.Publisher('finish_move', String, queue_size=10)

# Nodo para escuchar comandos
rospy.Subscriber('first_move', String, command_callback)
rospy.Subscriber('continue_move', String, command_callback)

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
