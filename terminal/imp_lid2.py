#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO
import time

# Inicializar nodo ROS
rospy.init_node('lidar_adjusted_movement', anonymous=True)

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

# Función para mover los motores
def move_motors(distancia_cm):
    global start_time, end_time, encoder_left_count, encoder_right_count
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(85)
    pwm_right.ChangeDutyCycle(92)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    print(f"Inicio del movimiento: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        print(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    print("Motores detenidos")

# Suscripción a la posición del Lidar
def pose_callback(data):
    rospy.loginfo("Received Lidar data: x={:.3f}, y={:.3f}, theta={:.2f}".format(data.x, data.y, data.theta))
    if data.y > 0.01:  # Desviación positiva mayor a 1 cm
        adjust_movement(data.y)
    elif data.y < -0.01:  # Desviación negativa mayor a 1 cm
        adjust_movement(data.y)
    else:
        # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
        pwm_left.ChangeDutyCycle(85)
        pwm_right.ChangeDutyCycle(92)

rospy.Subscriber('simple_pose', Pose2D, pose_callback)

def adjust_movement(y_deviation):
    if y_deviation > 0.01:  # Desviación positiva mayor a 1 cm
        pwm_left.ChangeDutyCycle(90)  # Ajustar duty cycle para corrección
        pwm_right.ChangeDutyCycle(84)
    elif y_deviation < -0.01:  # Desviación negativa mayor a 1 cm
        pwm_left.ChangeDutyCycle(84)
        pwm_right.ChangeDutyCycle(90)
    rospy.loginfo(f"Adjusting due to y deviation: {y_deviation:.3f}")

if __name__ == '__main__':
    try:
        move_motors(500)  # Mover 200 cm
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Movement finished")
        rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
