#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import json

# Inicializar nodo ROS
rospy.init_node('evade_obstacle', anonymous=True, log_level=rospy.DEBUG)

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

# Variables para detección de obstáculos
should_stop = False
obstacle_position_x = None

# Rango de detección de obstáculos (en metros)
detection_threshold = 0.40

# Frecuencia de ejecución
rate = rospy.Rate(4)  # 4 Hz

# Función de callback para los encoders
def encoder_callback_izq(channel):
    global encoder_left_count
    encoder_left_count += 1
    rospy.logdebug(f"Encoder izquierdo: {encoder_left_count}")

def encoder_callback_der(channel):
    global encoder_right_count
    encoder_right_count += 1
    rospy.logdebug(f"Encoder derecho: {encoder_right_count}")

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_IZQ, GPIO.RISING, callback=encoder_callback_izq)
GPIO.add_event_detect(ENCODER_DER, GPIO.RISING, callback=encoder_callback_der)

# Función de callback para la detección de obstáculos
def filtered_obstacles_callback(data):
    global should_stop, obstacle_position_x
    filtered_data = json.loads(data.data)
    for entry in filtered_data:
        distance_to_obstacle = entry['center_x']
        rospy.logdebug(f"Datos del obstáculo: center_x={distance_to_obstacle}")
        if distance_to_obstacle <= detection_threshold:
            should_stop = True
            obstacle_position_x = distance_to_obstacle
            rospy.loginfo(f"Obstáculo detectado dentro del rango: center_x={distance_to_obstacle}")
            return

# Función para suscribirse al tópico filtrado de obstáculos
def subscribe_to_filtered_obstacles():
    global obstacle_sub
    obstacle_sub = rospy.Subscriber('/filtered_obstacles', String, filtered_obstacles_callback)

# Función para mover el robot y verificar obstáculos
def move_straight_distance(distance_cm, pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count, should_stop, obstacle_position_x

    # Inicializar los encoders y las PWM
    encoder_left_count = 0
    encoder_right_count = 0
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)

    # Depuración
    rospy.logdebug("Motores activados: IN1=HIGH, IN2=LOW, IN3=HIGH, IN4=LOW")
    rospy.logdebug(f"Duty Cycle: pwm_left={pwm_left_speed}, pwm_right={pwm_right_speed}")
    
    # Calcular los pulsos deseados
    pulsos_deseados_izq = distance_cm * k_izq
    pulsos_deseados_der = distance_cm * k_der
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")

    try:
        while encoder_left_count < pulsos_deseados_izq and encoder_right_count < pulsos_deseados_der:
            rospy.logdebug(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")

            if should_stop:
                rospy.loginfo("Obstáculo detectado, deteniendo el movimiento")
                pwm_left.ChangeDutyCycle(0)
                pwm_right.ChangeDutyCycle(0)
                return  # Terminar el código después de detenerse

            rate.sleep()  # Mantener la frecuencia deseada
    except KeyboardInterrupt:
        rospy.loginfo("Interrupción de ROS detectada (Ctrl+C). Apagando el nodo.")
        pwm_left.ChangeDutyCycle(0)
        pwm_right.ChangeDutyCycle(0)
        GPIO.cleanup()
        rospy.signal_shutdown("Ctrl+C pressed. Shutting down.")

    # Detener los motores
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado: Motores detenidos")

# Suscribir al tópico filtrado de obstáculos al inicio
subscribe_to_filtered_obstacles()

if __name__ == '__main__':
    try:
        rospy.loginfo("Iniciando movimiento y evasión de obstáculos")
        move_straight_distance(100, 98, 86)  # Mover hacia adelante por 1 metro a una velocidad más lenta
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
