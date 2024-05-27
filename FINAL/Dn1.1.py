#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import RPi.GPIO as GPIO
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time

# Inicializar nodo ROS
rospy.init_node('advance_2m_with_dynamic_object_detection', anonymous=True)

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

# Clase para detección de objetos dinámicos
class DynamicObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.object_detected = False  # Variable para indicar si se ha detectado un objeto dinámico
        self.last_detection_time = 0  # Tiempo de la última detección

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_image()
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_image(self):
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Umbral para determinar si es un objeto significativo
                    self.object_detected = True
                    self.last_detection_time = time.time()
                    break

        # Dibujar los contornos detectados
        cv2.drawContours(self.cv_image, contours, -1, (0, 255, 0), 2)
        cv2.imshow("Camera View", self.cv_image)
        cv2.waitKey(1)

# Instancia del detector de objetos dinámicos
dynamic_object_detector = DynamicObjectDetector()

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

# Función para mover los motores en línea recta con detección de objetos dinámicos
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
        if dynamic_object_detector.object_detected:
            rospy.loginfo("Detenido por detección de objeto dinámico")
            time.sleep(2)  # Esperar 2 segundos antes de reanudar
            if time.time() - dynamic_object_detector.last_detection_time > 2:  # Si no se detecta un objeto en 2 segundos
                dynamic_object_detector.object_detected = False  # Restablecer la detección
            continue
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado o detenido por objeto dinámico: Motores detenidos")

    # Publicar los resultados finales
    rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Time elapsed: {end_time - start_time:.2f} seconds")

# Función para manejar el comando Dn1.1
def handle_Dn1_1():
    move_straight_2m()
    done_pub.publish("Done")
    rospy.loginfo("Mensaje 'Done' publicado")

def reset_state():
    global encoder_left_count, encoder_right_count
    encoder_left_count = 0
    encoder_right_count = 0
    dynamic_object_detector.object_detected = False
    dynamic_object_detector.last_detection_time = 0

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    reset_state()
    if data.data == "Dn1.1":
        handle_Dn1_1()

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
