#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO
import math

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

# Variables para almacenar los pulsos en cada fase
pulses_first_straight = [0, 0]  # [left, right]
pulses_curve = [0, 0]  # [left, right]
pulses_second_straight = [0, 0]  # [left, right]

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_IZQ, GPIO.RISING, callback=lambda channel: encoder_callback(channel, 'left'))
GPIO.add_event_detect(ENCODER_DER, GPIO.RISING, callback=lambda channel: encoder_callback(channel, 'right'))

def encoder_callback(channel, side):
    global encoder_left_count, encoder_right_count
    if side == 'left':
        encoder_left_count += 1
    elif side == 'right':
        encoder_right_count += 1
    rospy.loginfo(f"Encoder {side} count: Left {encoder_left_count}, Right {encoder_right_count}")

def move_distance(target_pulses, pwm_left_speed, pwm_right_speed, phase):
    initial_pulses_left = encoder_left_count
    initial_pulses_right = encoder_right_count

    rospy.loginfo(f"Starting movement for {target_pulses} pulses with PWM Left: {pwm_left_speed}%, Right: {pwm_right_speed}%.")
    GPIO.output([MOTOR_IZQ_IN1, MOTOR_DER_IN3], GPIO.HIGH)
    GPIO.output([MOTOR_IZQ_IN2, MOTOR_DER_IN4], GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)

    while True:
        current_pulses_left = encoder_left_count - initial_pulses_left
        current_pulses_right = encoder_right_count - initial_pulses_right
        current_pulses_avg = (current_pulses_left + current_pulses_right) / 2
        if current_pulses_avg >= target_pulses:
            break
        rospy.sleep(0.01)

    # Almacenar datos de pulsos basados en la fase
    if phase == 'first_straight':
        pulses_first_straight[0] = current_pulses_left
        pulses_first_straight[1] = current_pulses_right
    elif phase == 'curve':
        pulses_curve[0] = current_pulses_left
        pulses_curve[1] = current_pulses_right
    elif phase == 'second_straight':
        pulses_second_straight[0] = current_pulses_left
        pulses_second_straight[1] = current_pulses_right

    # Detener motores
    GPIO.output([MOTOR_IZQ_IN1, MOTOR_IZQ_IN2, MOTOR_DER_IN3, MOTOR_DER_IN4], GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def start_l_turn_movement():
    # Calcula los pulsos para las distancias rectas usando las constantes k
    distance_first_straight = 150  # Distancia en cm para la primera parte recta (1.5 metros)
    distance_second_straight = 150  # Distancia en cm para la segunda parte recta (1.5 metros)

    straight_initial_left_pulses = distance_first_straight * k_izq
    straight_initial_right_pulses = distance_first_straight * k_der
    straight_final_left_pulses = distance_second_straight * k_izq
    straight_final_right_pulses = distance_second_straight * k_der

    # Pulsos estimados para la curva, necesitan calibración
    curve_pulses = 486  # Este valor puede necesitar ajuste basado en pruebas reales

    # Realizar los movimientos
    move_distance(int(straight_initial_left_pulses), 85, 92, 'first_straight')  # Mover recto para la primera parte de 1.5 metros
    move_distance(curve_pulses, 90, 15, 'curve')  # Ajustar PWM para la curva
    move_distance(int(straight_final_left_pulses), 85, 92, 'second_straight')  # Mover recto para la parte final de 1.5 metros

    rospy.loginfo(f"Pulses count: First straight - Left {pulses_first_straight[0]}, Right {pulses_first_straight[1]}, "
                  f"Curve - Left {pulses_curve[0]}, Right {pulses_curve[1]}, "
                  f"Second straight - Left {pulses_second_straight[0]}, Right {pulses_second_straight[1]}")

if __name__ == '__main__':
    try:
        start_l_turn_movement()
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
