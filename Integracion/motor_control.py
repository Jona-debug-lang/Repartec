#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import math

# Inicializar nodo ROS
rospy.init_node('fixed_distance_motor_control', anonymous=True)

# Configuración de los GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definición de pines
ENABLE1, ENABLE2 = 12, 13
IN1, IN2, IN3, IN4 = 23, 24, 27, 22
ENCODER1, ENCODER2 = 16,26

# Configurar pines de motor y encoder
GPIO.setup([ENABLE1, ENABLE2, IN1, IN2, IN3, IN4], GPIO.OUT)
GPIO.setup([ENCODER1, ENCODER2], GPIO.IN)

# Configurar PWM
pwm_a = GPIO.PWM(ENABLE1, 1000)
pwm_b = GPIO.PWM(ENABLE2, 1000)
pwm_a.start(0)
pwm_b.start(0)

# Variables para el seguimiento de encoders
encoder_left_count = 0
encoder_right_count = 0 

def encoder_callback(channel):
    global encoder_left_count, encoder_right_count
    if channel == ENCODER1:
        encoder_left_count += 1
    elif channel == ENCODER2:
        encoder_right_count += 1
    rospy.loginfo(f"Encoder count: Left {encoder_left_count}, Right {encoder_right_count}")

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER1, GPIO.RISING, callback=encoder_callback)
GPIO.add_event_detect(ENCODER2, GPIO.RISING, callback=encoder_callback)

def start_movement(target_distance_cm):
    pulses_per_cm = 2.2# Ajustado según nueva relación
    target_pulses = int(target_distance_cm * pulses_per_cm)

    # Establece la dirección de avance
    GPIO.output([IN1, IN3], GPIO.HIGH)
    GPIO.output([IN2, IN4], GPIO.LOW)
    
    # Activa los motores con ciclo lento para mayor precisión
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)

    rospy.loginfo(f"Starting movement to cover {target_distance_cm} cm, requiring {target_pulses} pulses.")
    while True:
        average_pulses = (encoder_left_count + encoder_right_count) / 2
        rospy.loginfo(f"Current Pulses: Left {encoder_left_count}, Right {encoder_right_count}, Average: {average_pulses}")
        if average_pulses >= target_pulses:
            break
        rospy.sleep(0.01)  # Pequeña pausa para no sobrecargar el CPU

    # Detiene los motores
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    rospy.loginfo(f"Destination reached with {encoder_left_count} left pulses and {encoder_right_count} right pulses.")

if __name__ == '__main__':
    try:
        desired_distance_cm = 200  # Cambia aquí para probar con diferentes distancias
        start_movement(desired_distance_cm)
    finally:
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
