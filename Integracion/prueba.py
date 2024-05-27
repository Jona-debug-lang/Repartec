#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rospy

# Inicializar el nodo de ROS
rospy.init_node('encoder_test')

# Configurar los pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definir pines GPIO para el encoder
ENCODER_PIN_A = 19
ENCODER_PIN_B = 26

# Configurar pines del encoder
GPIO.setup(ENCODER_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables para guardar el estado del encoder
encoder_pos = 0

def encoder_callback(channel):
    global encoder_pos
    if GPIO.input(ENCODER_PIN_A) == GPIO.input(ENCODER_PIN_B):
        encoder_pos += 1
    else:
        encoder_pos -= 1
    rospy.loginfo(f'Posición del encoder: {encoder_pos}')

# Añadir detección de eventos a los pines del encoder
GPIO.add_event_detect(ENCODER_PIN_A, GPIO.BOTH, callback=encoder_callback)
GPIO.add_event_detect(ENCODER_PIN_B, GPIO.BOTH, callback=encoder_callback)

try:
    # Bucle para mantener el programa corriendo
    rospy.spin()
finally:
    GPIO.cleanup()
    rospy.loginfo("Limpieza de GPIO y cierre del nodo.")
