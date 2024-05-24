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

# Guardar datos de obstáculos en un archivo
def save_obstacle_data(filename, duration, moving=False):
    start_time = rospy.get_time()
    with open(filename, 'w') as file:
        while rospy.get_time() - start_time < duration:
            obstacles_msg = rospy.wait_for_message('/obstacles', Obstacles)
            log_msg = f"Tiempo: {rospy.get_time() - start_time:.2f}, Datos de obstáculos: {obstacles_msg}\n"
            rospy.loginfo(log_msg)
            file.write(log_msg)
            if moving:
                check_and_move()

# Función para mover el robot y verificar obstáculos
def check_and_move():
    pwm_left.ChangeDutyCycle(50)  # Velocidad reducida
    pwm_right.ChangeDutyCycle(50)
    time.sleep(0.1)  # Mover durante 0.1 segundos por cada ciclo

if __name__ == '__main__':
    try:
        rospy.loginfo("Iniciando prueba de captura de datos estáticos.")
        save_obstacle_data('datos_estaticos.txt', 5)  # Guardar datos mientras estático por 5 segundos
        
        rospy.loginfo("Iniciando prueba de captura de datos en movimiento.")
        save_obstacle_data('datos_movimiento.txt', 10, moving=True)  # Guardar datos mientras en movimiento por 10 segundos
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
