#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO
import time

# Inicializar nodo ROS
rospy.init_node('trajectory_control', anonymous=True)

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
deviation_x = 0
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
    global deviation_y, deviation_x, current_x
    deviation_y = data.y
    deviation_x = data.x
    current_x = data.x
    rospy.loginfo("Received Lidar data: x={:.3f}, y={:.3f}, theta={:.2f}".format(data.x, data.y, data.theta))

rospy.Subscriber('simple_pose', Pose2D, pose_callback)

# Función para ajustar el movimiento basado en la desviación del Lidar hacia adelante
def adjust_movement_forward(deviation, axis):
    if axis == 'y':
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
    elif axis == 'x':
        if deviation > 0.08:  # Desviación positiva mayor a 8 cm
            pwm_left.ChangeDutyCycle(90)
            pwm_right.ChangeDutyCycle(80)
        elif deviation < -0.08:  # Desviación negativa mayor a 8 cm
            pwm_left.ChangeDutyCycle(80)
            pwm_right.ChangeDutyCycle(90)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 8 cm
            pwm_left.ChangeDutyCycle(85)
            pwm_right.ChangeDutyCycle(85)
    rospy.loginfo(f"Adjusting due to {axis.upper()} deviation: {deviation:.3f}")

# Función para ajustar el movimiento basado en la desviación del Lidar hacia atrás
def adjust_movement_backward(deviation):
    if deviation > 0.08:  # Desviación positiva mayor a 8 cm
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(95)
    elif deviation < -0.08:  # Desviación negativa mayor a 8 cm
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(80)
    else:
        # Restablecer los PWM a valores normales si la desviación es menor o igual a 8 cm
        pwm_left.ChangeDutyCycle(96)
        pwm_right.ChangeDutyCycle(95)
    rospy.loginfo(f"Adjusting due to Y deviation (backward): {deviation:.3f}")

# Función para mover los motores en línea recta con corrección
def move_straight(distancia_cm, pwm_left_speed, pwm_right_speed, axis):
    global start_time, end_time, encoder_left_count, encoder_right_count, current_x
    
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
        if axis == 'y':
            adjust_movement_forward(deviation_y, 'y')
        elif axis == 'x':
            adjust_movement_forward(deviation_x, 'x')
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

    # Publicar los resultados finales
    rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Time elapsed: {end_time - start_time:.2f} seconds")

# Función para girar 90 grados
def turn_90_degrees(pulses_turn, pwm_left_speed, pwm_right_speed):
    global start_time, end_time, encoder_left_count, encoder_right_count
    
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    rospy.loginfo(f"Inicio del giro de 90 grados: Pulsos deseados: {pulses_turn}")
    
    while encoder_left_count < pulses_turn:
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Giro de 90 grados completado")
    
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
    
    rospy.loginfo(f"Inicio del giro en su lugar: Pulsos deseados: {pulses_turn}")
    
    while encoder_left_count < pulses_turn:
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Giro en su lugar completado")
    
    # Publicar los resultados finales
    rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
    rospy.loginfo(f"Time elapsed: {end_time - start_time:.2f} seconds")

# Función para manejar la trayectoria T1
def handle_T1():
    move_straight(200, 97, 87, 'y')
    done_pub.publish("Forward Done")
    rospy.loginfo("Mensaje 'Forward Done' publicado")
    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    move_straight_2m_backward()
    done_pub.publish("Backward Done")
    rospy.loginfo("Mensaje 'Backward Done' publicado")

# Función para manejar la trayectoria T2
def handle_T2():
    # Primera parte recta (70 cm) - Corrección en eje Y
    move_straight(70, 85, 100, 'y')
    
    # Primera curva de 90 grados
    turn_90_degrees(5000, 100, 0)
    
    # Segunda parte recta (30 cm) - Corrección en eje X
    move_straight(40, 85, 100, 'x')
    
    # Segunda curva de 90 grados
    turn_90_degrees(6750, 100, 0)
    
    # Tercera parte recta (45 cm) - Corrección en eje Y
    move_straight(45, 85, 100, 'y')
    
    done_pub.publish("T2 Part 1 Done")
    rospy.loginfo("Mensaje 'T2 Part 1 Done' publicado")
    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    
    # Segunda parte de la trayectoria T2
    # Salida a terreno ideal
    move_straight(60, 70, 80, 'y')
    
    # Vuelta a 90 grados en su propio eje en terreno ideal
    turn_in_place_90_degrees(800, 50)
    
    # Recta en terreno ideal
    move_straight(200, 55, 75, 'x')
    
    # Segunda vuelta de 90 grados en su propio eje y final
    turn_in_place_90_degrees(800, 50)
    
    # Regreso a terreno anterior
    move_straight(60, 70, 80, 'y')
    
    done_pub.publish("T2 Done")
    rospy.loginfo("Mensaje 'T2 Done' publicado")

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
    elif data.data == "T2":
        handle_T2()

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
