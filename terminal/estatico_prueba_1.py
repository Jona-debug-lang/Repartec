#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
import RPi.GPIO as GPIO
import time

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
obstacle_position = None

# Rango de detección de obstáculos (en metros)
detection_range_min = 0.15
detection_range_max = 0.20

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
def obstacles_callback(data):
    global should_stop, obstacle_position
    for circle in data.circles:
        distance_to_obstacle = (circle.center.x ** 2 + circle.center.y ** 2) ** 0.5
        rospy.logdebug(f"Datos del círculo: posición=({circle.center.x}, {circle.center.y}), radio={circle.radius}, distancia={distance_to_obstacle}")
        if detection_range_min <= distance_to_obstacle <= detection_range_max:  # Ajustar según el tamaño y posición del termo
            should_stop = True
            obstacle_position = (circle.center.x, circle.center.y)
            rospy.loginfo(f"Obstáculo detectado dentro del rango: posición=({circle.center.x}, {circle.center.y}), distancia={distance_to_obstacle}")
            return

# Función para suscribirse al tópico de obstáculos
def subscribe_to_obstacles():
    rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

# Función para mover el robot y verificar obstáculos
def move_straight_distance(distance_cm, pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count, should_stop, obstacle_position

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

    while encoder_left_count < pulsos_deseados_izq and encoder_right_count < pulsos_deseados_der:
        rospy.logdebug(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")

        if should_stop:
            rospy.loginfo("Obstáculo detectado, deteniendo para analizar")
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            time.sleep(2)  # Pausa para analizar el obstáculo
            evade_obstacle()
            should_stop = False
            obstacle_position = None  # Reset obstacle position
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)

        time.sleep(0.01)  # Ajusta este valor según sea necesario

    # Detener los motores
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado: Motores detenidos")

# Función para evadir el obstáculo con curvas suaves
def evade_obstacle():
    global obstacle_position
    if obstacle_position:
        rospy.loginfo(f"Evitando obstáculo en posición: {obstacle_position}")
        # Desactivar la suscripción para evitar ruido durante la evasión
        obstacle_sub.unregister()
        
        # Implementa la lógica para evadir el obstáculo con una curva suave
        if obstacle_position[1] > 0:  # Obstáculo a la derecha
            # Girar a la izquierda suavemente
            curve_left(1500, 40, 60)  # Ajustar pulsos y velocidad para curva más lenta
            move_straight_distance(20, 45, 50)  # Mover 20 cm hacia adelante
            curve_right(1500, 60, 40)  # Girar a la derecha suavemente para volver a la ruta
        else:  # Obstáculo a la izquierda
            # Girar a la derecha suavemente
            curve_right(1500, 60, 40)  # Ajustar pulsos y velocidad para curva más lenta
            move_straight_distance(20, 45, 50)  # Mover 20 cm hacia adelante
            curve_left(1500, 40, 60)  # Girar a la izquierda suavemente para volver a la ruta

        # Reactivar la suscripción después de la evasión
        subscribe_to_obstacles()

# Función para realizar una curva a la izquierda suavemente
def curve_left(pulses_turn, pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)

    encoder_left_count = 0
    encoder_right_count = 0
    rospy.loginfo(f"Iniciando curva a la izquierda: Pulsos deseados izq: {pulses_turn}")
    
    while encoder_left_count < pulses_turn and encoder_right_count < pulses_turn:
        rospy.logdebug(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario

    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Curva a la izquierda completada: Motores detenidos")

# Función para realizar una curva a la derecha suavemente
def curve_right(pulses_turn, pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)

    encoder_left_count = 0
    encoder_right_count = 0
    rospy.loginfo(f"Iniciando curva a la derecha: Pulsos deseados izq: {pulses_turn}")
    
    while encoder_left_count < pulses_turn and encoder_right_count < pulses_turn:
        rospy.logdebug(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario

    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Curva a la derecha completada: Motores detenidos")

# Suscribir al tópico de obstáculos al inicio
obstacle_sub = rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

if __name__ == '__main__':
    try:
        rospy.loginfo("Iniciando movimiento y evasión de obstáculos")
        move_straight_distance(200, 40, 40)  # Mover hacia adelante por 2 metros a una velocidad más lenta
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
