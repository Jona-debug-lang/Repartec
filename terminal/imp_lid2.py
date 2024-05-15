#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO

# Inicializar nodo ROS
rospy.init_node('lidar_adjusted_movement', anonymous=True)

# Configuración de los GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definición de pines
ENABLE1, ENABLE2 = 12, 13
IN1, IN2, IN3, IN4 = 23, 24, 27, 22
ENCODER_LEFT_A, ENCODER_LEFT_B = 5, 16
ENCODER_RIGHT_A, ENCODER_RIGHT_B = 6, 26

# Configurar pines de motor y encoder
GPIO.setup([ENABLE1, ENABLE2, IN1, IN2, IN3, IN4], GPIO.OUT)
GPIO.setup([ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B], GPIO.IN)

# Configurar PWM
pwm_left = GPIO.PWM(ENABLE1, 1000)
pwm_right = GPIO.PWM(ENABLE2, 1000)
pwm_left.start(0)
pwm_right.start(0)

# Variables para el seguimiento de encoders
encoder_left_count = 0
encoder_right_count = 0
encoder_left_last_state_A = GPIO.input(ENCODER_LEFT_A)
encoder_right_last_state_A = GPIO.input(ENCODER_RIGHT_A)

# Función de callback para los encoders
def encoder_callback_left(channel):
    global encoder_left_count, encoder_left_last_state_A
    state_A = GPIO.input(ENCODER_LEFT_A)
    state_B = GPIO.input(ENCODER_LEFT_B)
    if state_A != encoder_left_last_state_A:
        if state_A == state_B:
            encoder_left_count += 1
        else:
            encoder_left_count -= 1
        encoder_left_last_state_A = state_A
    rospy.loginfo(f"Encoder Left count: {encoder_left_count}")

def encoder_callback_right(channel):
    global encoder_right_count, encoder_right_last_state_A
    state_A = GPIO.input(ENCODER_RIGHT_A)
    state_B = GPIO.input(ENCODER_RIGHT_B)
    if state_A != encoder_right_last_state_A:
        if state_A == state_B:
            encoder_right_count += 1
        else:
            encoder_right_count -= 1
        encoder_right_last_state_A = state_A
    rospy.loginfo(f"Encoder Right count: {encoder_right_count}")

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(ENCODER_LEFT_B, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.RISING, callback=encoder_callback_right)
GPIO.add_event_detect(ENCODER_RIGHT_B, GPIO.RISING, callback=encoder_callback_right)

# Suscripción a la posición del Lidar
def pose_callback(data):
    rospy.loginfo("Received Lidar data: x={:.3f}, y={:.3f}, theta={:.2f}".format(data.x, data.y, data.theta))
    if abs(data.y) > 0.01:  # Más de 1 cm de desviación en Y
        adjust_movement(data.y)
    else:
        # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
        pwm_left.ChangeDutyCycle(100)
        pwm_right.ChangeDutyCycle(100)

rospy.Subscriber('simple_pose', Pose2D, pose_callback)

def adjust_movement(y_deviation):
    if y_deviation > 0.01:  # Desviación positiva mayor a 1 cm
        pwm_left.ChangeDutyCycle(95)  # Ajustar duty cycle para corrección
        pwm_right.ChangeDutyCycle(90)
    elif y_deviation < -0.01:  # Desviación negativa mayor a 1 cm
        pwm_left.ChangeDutyCycle(90)
        pwm_right.ChangeDutyCycle(95)
    rospy.loginfo(f"Adjusting due to y deviation: {y_deviation:.3f}")

def move_straight(target_distance_cm):
    pulses_per_cm = 2.58  # Usar la calibración que ya tienes
    target_pulses = int(target_distance_cm * pulses_per_cm)
    initial_pulses = (encoder_left_count + encoder_right_count) // 2

    rospy.loginfo("Starting straight movement.")
    GPIO.output([IN1, IN3], GPIO.HIGH)
    GPIO.output([IN2, IN4], GPIO.LOW)
    pwm_left.ChangeDutyCycle(100)
    pwm_right.ChangeDutyCycle(100)

    while True:
        current_pulses = (encoder_left_count + encoder_right_count) // 2
        rospy.loginfo(f"Current Pulses: Left {encoder_left_count}, Right {encoder_right_count}, Average: {current_pulses}")
        if abs(current_pulses - initial_pulses) >= target_pulses:
            break
        rospy.sleep(0.01)

    # Detener motores
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo(f"Destination reached with {encoder_left_count} left pulses and {encoder_right_count} right pulses.")

if __name__ == '__main__':
    try:
        move_straight(200)  # Mover 200 cm
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Movement finished")
        rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
