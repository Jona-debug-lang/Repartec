#!/usr/bin/env python3
import rospy
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

# Función para mover los motores en línea recta con corrección de ruta
def move_straight(distancia_cm, pwm_left_speed, pwm_right_speed, eje='y'):
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
    
    print(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        print(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        adjust_movement(deviation_y if eje == 'y' else deviation_x, eje)
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    print("Motores detenidos")

# Función para girar 90 grados
def turn_90_degrees(pwm_left_speed, pwm_right_speed):
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
    
    pulsos_deseados_izq = 5230  # Pulsos para 90 grados a la izquierda
    pulsos_deseados_der = 4290  # Pulsos para 90 grados a la derecha
    
    print(f"Inicio de la curva: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        print(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    
    # Obtener theta final del LIDAR
    final_theta = lidar_data[-1][2] if lidar_data else 0
    theta_change = final_theta - initial_theta
    theta_changes.append(theta_change)
    
    print("Motores detenidos")

# Función para ajustar el movimiento basado en la desviación del Lidar
def adjust_movement(deviation, eje='y'):
    if eje == 'y':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(90)  # Ajustar duty cycle para corrección
            pwm_right.ChangeDutyCycle(84)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(84)
            pwm_right.ChangeDutyCycle(90)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(85)
            pwm_right.ChangeDutyCycle(92)
    elif eje == 'x':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(84)
            pwm_right.ChangeDutyCycle(90)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(90)
            pwm_right.ChangeDutyCycle(84)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(85)
            pwm_right.ChangeDutyCycle(92)
    rospy.loginfo(f"Adjusting due to {eje} deviation: {deviation:.3f}")

def start_l_turn_movement():
    # Primera parte recta (110 cm) - Corrección en eje Y
    distancia_cm = 110
    move_straight(distancia_cm, 85, 92, eje='y')
    
    # Primera curva de 90 grados
    turn_90_degrees(100, 70)
    
    # Segunda parte recta (30 cm) - Corrección en eje X
    distancia_cm = 30
    move_straight(distancia_cm, 85, 92, eje='x')
    
    # Segunda curva de 90 grados
    turn_90_degrees(100, 70)

    # Tercera parte recta (110 cm) - Corrección en eje Y
    distancia_cm = 110
    move_straight(distancia_cm, 85, 92, eje='y')

if __name__ == '__main__':
    try:
        start_l_turn_movement()
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
        rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
        rospy.loginfo(f"Lidar data collected: {lidar_data}")
        rospy.loginfo(f"Theta changes during turns: {theta_changes}")



#chmod +x ultimo_curva.py
#rosrun my_python_scripts ultimo_curva.py