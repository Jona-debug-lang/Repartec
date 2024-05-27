#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO
import time

# Inicializar nodo ROS
rospy.init_node('l_turn_movement_detailed', anonymous=True)

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

# Variables para la corrección de ruta
deviation_y = 0
deviation_x = 0
lidar_data = []
theta_changes = []

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
    global deviation_y, deviation_x
    deviation_y = data.y
    deviation_x = data.x
    lidar_data.append((data.x, data.y, data.theta))
    rospy.loginfo("Received Lidar data: x={:.3f}, y={:.3f}, theta={:.2f}".format(data.x, data.y, data.theta))

rospy.Subscriber('simple_pose', Pose2D, pose_callback)

# Publicador para el mensaje "finish_move"
done_pub = rospy.Publisher('finish_move', String, queue_size=10)

# Publicador para el mensaje "finish_move"
boton = rospy.Publisher('boton', String, queue_size=10)

# Función para mover los motores en línea recta con corrección de ruta
def move_straight(distancia_cm, pwm_left_speed, pwm_right_speed, eje):
    global start_time, end_time, encoder_left_count, encoder_right_count, deviation_y, deviation_x
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
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        adjust_movement(deviation_y if eje == 'y' else deviation_x, eje)  # Llamada correcta a la función
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

# Función para girar 90 grados
def turn_90_degrees(pulses_turn, pwm_left_speed, pwm_right_speed):
    global start_time, end_time, encoder_left_count, encoder_right_count
    initial_theta = lidar_data[-1][2] if lidar_data else 0  # Obtener theta inicial del LIDAR
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = pulses_turn  # Pulsos para 90 grados a la izquierda
    
    rospy.loginfo(f"Inicio de la curva: Pulsos deseados izq: {pulsos_deseados_izq}")
    
    while encoder_left_count < pulsos_deseados_izq:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    
    # Obtener theta final del LIDAR
    final_theta = lidar_data[-1][2] if lidar_data else 0
    theta_change = final_theta - initial_theta
    theta_changes.append(theta_change)
    
    rospy.loginfo("Motores detenidos")

# Función para girar 90 grados en su propio eje
def turn_in_place_90_degrees(pulses_turn, pwm_speed):
    global start_time, end_time, encoder_left_count, encoder_right_count
    initial_theta = lidar_data[-1][2] if lidar_data else 0  # Obtener theta inicial del LIDAR
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(pwm_speed)
    pwm_right.ChangeDutyCycle(pwm_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados = pulses_turn  # Ajusta este valor según tus necesidades
    
    rospy.loginfo(f"Inicio de la curva en su lugar: Pulsos deseados: {pulsos_deseados}")
    
    while encoder_left_count < pulsos_deseados:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    
    # Obtener theta final del LIDAR
    final_theta = lidar_data[-1][2] if lidar_data else 0
    theta_change = final_theta - initial_theta
    theta_changes.append(theta_change)
    
    rospy.loginfo("Motores detenidos")

# Función para ajustar el movimiento basado en la desviación del Lidar
def adjust_movement(deviation, eje):
    if eje == 'y':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(70)  # Ajustar duty cycle para corrección
            pwm_right.ChangeDutyCycle(100)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(75)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(100)
    elif eje == 'x':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(70)
            pwm_right.ChangeDutyCycle(100)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(75)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(100)
    rospy.loginfo(f"Adjusting due to {eje} deviation: {deviation:.3f}")

# Función para manejar el comando T1
def handle_T1():
    # Primera parte recta (70 cm) - Corrección en eje Y
    distancia_cm = 70
    move_straight(distancia_cm, 85, 100, 'y')  # Velocidades calibradas
    
    # Primera curva de 90 grados
    turn_90_degrees(5000, 100, 0)
    
    # Segunda parte recta (30 cm) - Corrección en eje X
    distancia_cm = 40
    move_straight(distancia_cm, 85, 100, 'x')  # Velocidades calibradas
    
    # Segunda curva de 90 grados
    turn_90_degrees(6750, 100, 0)
    
    # Tercera parte recta (45 cm) - Corrección en eje Y
    distancia_cm = 45
    move_straight(distancia_cm, 85, 100, 'y')  # Velocidades calibradas

    boton.publish("GRACIAS")

    rospy.loginfo("Esperando mensaje continue_move...")
    time.sleep(0.1)  # Espera corta para asegurarse de que el búfer esté limpio
    rospy.wait_for_message('continue_move', String)

    # Salida a terreno ideal
    distancia_cm = 60
    move_straight(distancia_cm, 70, 80, 'y')  # Velocidad reducida

    # Vuelta a 90 grados en su propio eje en terreno ideal
    turn_in_place_90_degrees(800, 50)  # Velocidad reducida

    # Recta en terreno ideal
    distancia_cm = 200
    move_straight(distancia_cm,55, 75, 'x')  # Velocidad reducida

    # Segunda vuelta de 90 grados en su propio eje y final
    turn_in_place_90_degrees(800, 50)  # Velocidad reducida

    # Regreso a terreno anterior
    distancia_cm = 60
    move_straight(distancia_cm, 70, 80, 'y')  # Velocidad reducida

# Función para manejar el comando T2
def handle_T2():
    # Mover 200 cm hacia adelante
    distancia_cm = 200
    move_straight(distancia_cm, 100, 100, 'y')  # Velocidades calibradas
    
    boton.publish("GRACIAS")
    rospy.loginfo("Esperando mensaje continue_move...")
    time.sleep(0.1)  # Espera corta para asegurarse de que el búfer esté limpio
    rospy.wait_for_message('continue_move', String)
    
    # Mover 200 cm en reversa
    GPIO.output(MOTOR_IZQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(100)
    pwm_right.ChangeDutyCycle(100)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento reversa: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        adjust_movement(deviation_y if eje == 'y' else deviation_x, eje)  # Llamada correcta a la función
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

def reset_state():
    global encoder_left_count, encoder_right_count, deviation_y, deviation_x, lidar_data, theta_changes
    encoder_left_count = 0
    encoder_right_count = 0
    deviation_y = 0
    deviation_x = 0
    lidar_data = []
    theta_changes = []

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    reset_state()
    if data.data == "T1":
        handle_T1()
    elif data.data == "T2":
        handle_T2()
    done_pub.publish("Done")
    rospy.loginfo("Mensaje 'Done' publicado")

# Nodo para escuchar comandos
rospy.Subscriber('first_move', String, command_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
        rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
        rospy.loginfo(f"Lidar data collected: {lidar_data}")
        rospy.loginfo(f"Theta changes during turns: {theta_changes}")

