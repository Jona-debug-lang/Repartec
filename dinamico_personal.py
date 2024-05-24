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

# Guardar datos de obstáculos en un archivo
def save_obstacle_data(filename, moving_distance_cm, moving=False):
    start_time = rospy.get_time()
    with open(filename, 'w') as file:
        if moving:
            move_straight_distance(moving_distance_cm, 50, 50, file)
        else:
            while rospy.get_time() - start_time < 5:  # Guardar datos por 5 segundos estático
                obstacles_msg = rospy.wait_for_message('/obstacles', Obstacles)
                log_msg = f"Tiempo: {rospy.get_time() - start_time:.2f}, Datos de obstáculos: {obstacles_msg}\n"
                rospy.loginfo(log_msg)
                file.write(log_msg)

# Función para mover el robot y verificar obstáculos
def move_straight_distance(distance_cm, pwm_left_speed, pwm_right_speed, file):
    global encoder_left_count, encoder_right_count, should_stop

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
        obstacles_msg = rospy.wait_for_message('/obstacles', Obstacles)
        log_msg = f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}, Datos de obstáculos: {obstacles_msg}\n"
        rospy.loginfo(log_msg)
        file.write(log_msg)
        time.sleep(0.01)  # Ajusta este valor según sea necesario

    # Detener los motores
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado: Motores detenidos")

if __name__ == '__main__':
    try:
        rospy.loginfo("Iniciando prueba de captura de datos estáticos.")
        save_obstacle_data('datos_estaticos.txt', 0, moving=False)  # Guardar datos mientras estático por 5 segundos
        
        rospy.loginfo("Iniciando prueba de captura de datos en movimiento.")
        save_obstacle_data('datos_movimiento.txt', 200, moving=True)  # Guardar datos mientras en movimiento por 2 metros
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
