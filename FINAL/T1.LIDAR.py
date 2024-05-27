#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO
import time

# Inicializar nodo ROS
rospy.init_node('forward_and_reverse_2m', anonymous=True)

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

# Función para ajustar el movimiento basado en la desviación del Lidar
def adjust_movement(deviation):
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
    rospy.loginfo(f"Adjusting due to Y deviation: {deviation:.3f}")

# Función para mover los motores en línea recta hacia adelante con corrección
def move_straight_2m_forward():
    global start_time, end_time, encoder_left_count, encoder_right_count, current_x
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(97)
    pwm_right.ChangeDutyCycle(87)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = 200 * k_izq  # 2 metros * k_izq
    pulsos_deseados_der = 200 * k_der  # 2 metros * k_der
    
    rospy.loginfo("Inicio del movimiento recto de 2 metros hacia adelante")
    
    while (encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der) and current_x > -1.75:
        adjust_movement(deviation_y)
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")
    
    # Publicar los resultados finales
    rospy.loginfo(f"Forward counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Forward time elapsed: {end_time - start_time:.2f} seconds")

# Función para mover los motores en línea recta hacia atrás
def move_straight_2m_backward():
    global start_time, end_time, encoder_left_count, encoder_right_count, current_x
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(96)
    pwm_right.ChangeDutyCycle(95)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = 200 * k_izq  # 2 metros * k_izq
    pulsos_deseados_der = 200 * k_der  # 2 metros * k_der
    
    rospy.loginfo("Inicio del movimiento recto de 2 metros hacia atrás")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")
    
    # Publicar los resultados finales
    rospy.loginfo(f"Backward counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Backward time elapsed: {end_time - start_time:.2f} seconds")

# Función para manejar el comando T1
def handle_T1():
    move_straight_2m_forward()
    done_pub.publish("Forward Done")
    rospy.loginfo("Mensaje 'Forward Done' publicado")
    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    move_straight_2m_backward()
    done_pub.publish("Backward Done")
    rospy.loginfo("Mensaje 'Backward Done' publicado")

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
