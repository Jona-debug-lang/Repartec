#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
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

# Variable global para control de detención
should_stop = False

# Publicador para los datos de obstáculos
obstacle_data_pub = rospy.Publisher('obstacle_data', String, queue_size=10)

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

# Función para verificar obstáculos
def check_obstacles():
    global should_stop
    should_stop = False
    obstacles_msg = rospy.wait_for_message('/obstacles', Obstacles)
    rospy.loginfo("Datos de obstáculos recibidos")
    for circle in obstacles_msg.circles:
        rospy.loginfo(f"Obstáculo detectado: posición=({circle.center.x}, {circle.center.y}), velocidad=({circle.velocity.x}, {circle.velocity.y}), radio={circle.radius}")
        distance = (circle.center.x ** 2 + circle.center.y ** 2) ** 0.5
        if 0.1 <= distance <= 2.0:  # Detenerse si el obstáculo está entre 10 cm y 2 metros
            should_stop = True
            rospy.loginfo(f"Detenerse: Objeto detectado a distancia {distance:.2f} metros")
            return

# Función para mover los motores en línea recta
def move_straight_2m(pwm_left_speed, pwm_right_speed):
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

if __name__ == '__main__':
    try:
        MAX_SPEED = 0.25  # Velocidad máxima para detenerse
        DETECTION_RADIUS = 2.0  # Radio de detección para detenerse
        rospy.loginfo("Ejecutando movimiento recto con detección de obstáculos")
        move_straight_2m(97, 87)  # Llamar a la función con las velocidades deseadas
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
