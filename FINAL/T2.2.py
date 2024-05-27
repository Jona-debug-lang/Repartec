#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO
import time

# Inicializar nodo ROS
rospy.init_node('trajectory_t2_2', anonymous=True)

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

# Función para mover los motores en línea recta
def move_straight(distancia_cm, pwm_left_speed, pwm_right_speed):
    global start_time, end_time, encoder_left_count, encoder_right_count
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

    # Publicar los resultados finales
    rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Time elapsed: {end_time - start_time:.2f} seconds")

# Función para girar 90 grados en su propio eje
def turn_in_place_90_degrees(pulses_turn, pwm_speed):
    global start_time, end_time, encoder_left_count, encoder_right_count
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(pwm_speed)
    pwm_right.ChangeDutyCycle(pwm_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados = pulses_turn
    
    rospy.loginfo(f"Inicio de la curva en su lugar: Pulsos deseados: {pulsos_deseados}")
    
    while encoder_left_count < pulsos_deseados:
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

    # Publicar los resultados finales
    rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Time elapsed: {end_time - start_time:.2f} seconds")

# Función para manejar el comando T2.2
def handle_T2_2():
    # Salida a terreno ideal
    distancia_cm = 60
    move_straight(distancia_cm, 70, 80)``
    
    # Vuelta a 90 grados en su propio eje en terreno ideal
    turn_in_place_90_degrees(800, 50)
    
    # Recta en terreno ideal
    distancia_cm = 200
    move_straight(distancia_cm, 55, 75)
    
    # Segunda vuelta de 90 grados en su propio eje y final
    turn_in_place_90_degrees(800, 50)
    
    # Regreso a terreno anterior
    distancia_cm = 60
    move_straight(distancia_cm, 70, 80)

    done_pub.publish("Done")
    rospy.loginfo("Mensaje 'Done' publicado")

def reset_state():
    global encoder_left_count, encoder_right_count
    encoder_left_count = 0
    encoder_right_count = 0

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    reset_state()
    if data.data == "T2.2":
        handle_T2_2()

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
