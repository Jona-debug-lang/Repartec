#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles  # Asumiendo que estás usando el mensaje Obstacles del paquete obstacle_detector
import RPi.GPIO as GPIO
import time

# Inicializar nodo ROS
rospy.init_node('straight_movement_with_obstacle_detection', anonymous=True)

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

# Variables para evasión de obstáculos
should_stop = False

# Función de callback para los encoders
def encoder_callback_izq(channel):
    global encoder_left_count
    encoder_left_count += 1
    rospy.loginfo(f"Encoder izquierdo: {encoder_left_count}")

def encoder_callback_der(channel):
    global encoder_right_count
    encoder_right_count += 1
    rospy.loginfo(f"Encoder derecho: {encoder_right_count}")

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_IZQ, GPIO.RISING, callback=encoder_callback_izq)
GPIO.add_event_detect(ENCODER_DER, GPIO.RISING, callback=encoder_callback_der)

# Función para verificar obstáculos
def check_obstacles(max_speed, detection_radius):
    global should_stop
    should_stop = False
    obstacles_msg = rospy.wait_for_message('/obstacles', Obstacles)
    rospy.loginfo("Datos de obstáculos recibidos")
    for circle in obstacles_msg.circles:
        rospy.loginfo(f"Obstáculo detectado: posición=({circle.center.x}, {circle.center.y}), velocidad=({circle.velocity.x}, {circle.velocity.y}), radio={circle.radius}")
        if abs(circle.velocity.x) > max_speed or abs(circle.velocity.y) > max_speed:
            # Verificar si el objeto en movimiento está dentro del radio de detección ajustable
            if (circle.center.x ** 2 + circle.center.y ** 2) <= detection_radius ** 2:
                should_stop = True
                rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
                return

# Función para mover los motores en línea recta
def move_straight_2m(pwm_left_speed, pwm_right_speed, max_speed, detection_radius):
    global encoder_left_count, encoder_right_count, should_stop
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    
    distancia_cm = 200
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        check_obstacles(max_speed, detection_radius)  # Verificar obstáculos en cada iteración
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Motores detenidos temporalmente")
            while should_stop:
                check_obstacles(max_speed, detection_radius)
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)
            rospy.loginfo("Movimiento reanudado")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado: Motores detenidos")

if __name__ == '__main__':
    try:
        move_straight_2m(100, 100, max_speed=0.1, detection_radius=1.0)  # Llamar a la función con las velocidades y parámetros deseados
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
