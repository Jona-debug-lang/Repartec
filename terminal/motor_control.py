#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import math

# Inicializar nodo ROS
rospy.init_node('l_turn_movement_detailed', anonymous=True)

# Configuración de los GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definición de pines
ENABLE1, ENABLE2 = 12, 13
IN1, IN2, IN3, IN4 = 23, 24, 27, 22
ENCODER_RIGHT, ENCODER_LEFT = 5, 6

# Configurar pines de motor y encoder
GPIO.setup([ENABLE1, ENABLE2, IN1, IN2, IN3, IN4], GPIO.OUT)
GPIO.setup([ENCODER_RIGHT, ENCODER_LEFT], GPIO.IN)

# Configurar PWM
pwm_right = GPIO.PWM(ENABLE1, 1000)
pwm_left = GPIO.PWM(ENABLE2, 1000)
pwm_right.start(0)
pwm_left.start(0)

# Variables para el seguimiento de encoders
encoder_right_count = 0
encoder_left_count = 0

# Variables para almacenar los pulsos en cada fase
pulses_first_straight = [0, 0]  # [right, left]
pulses_curve = [0, 0]  # [right, left]
pulses_second_straight = [0, 0]  # [right, left]

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_RIGHT, GPIO.RISING, callback=lambda channel: encoder_callback(channel, 'right'))
GPIO.add_event_detect(ENCODER_LEFT, GPIO.RISING, callback=lambda channel: encoder_callback(channel, 'left'))

def encoder_callback(channel, side):
    global encoder_right_count, encoder_left_count
    if side == 'right':
        encoder_right_count += 1
    elif side == 'left':
        encoder_left_count += 1
    rospy.loginfo(f"Encoder {side} count: Right {encoder_right_count}, Left {encoder_left_count}")

def move_distance(target_pulses, pwm_right_speed, pwm_left_speed, phase):
    initial_pulses_right = encoder_right_count
    initial_pulses_left = encoder_left_count

    rospy.loginfo(f"Starting movement for {target_pulses} pulses with PWM Right: {pwm_right_speed}%, Left: {pwm_left_speed}%.")
    GPIO.output([IN1, IN3], GPIO.HIGH)
    GPIO.output([IN2, IN4], GPIO.LOW)
    pwm_right.ChangeDutyCycle(pwm_right_speed)
    pwm_left.ChangeDutyCycle(pwm_left_speed)

    while True:
        current_pulses_right = encoder_right_count - initial_pulses_right
        current_pulses_left = encoder_left_count - initial_pulses_left
        current_pulses_avg = (current_pulses_right + current_pulses_left) / 2
        if current_pulses_avg >= target_pulses:
            break
        rospy.sleep(0.01)

    # Store pulse data based on phase
    if phase == 'first_straight':
        pulses_first_straight[0] = current_pulses_right
        pulses_first_straight[1] = current_pulses_left
    elif phase == 'curve':
        pulses_curve[0] = current_pulses_right
        pulses_curve[1] = current_pulses_left
    elif phase == 'second_straight':
        pulses_second_straight[0] = current_pulses_right
        pulses_second_straight[1] = current_pulses_left

    # Stop motors
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwm_right.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)

def start_l_turn_movement():
    pulses_per_cm = 7.455  # Adjust based on calibration
    straight_initial = 33 * pulses_per_cm
    straight_final = 33 * pulses_per_cm

    # Estimated curve pulses, needs calibration
    curve_pulses = 486

    # Perform the movements
    move_distance(int(straight_initial), 80, 80, 'first_straight')  # Move straight for the initial part of X
    move_distance(curve_pulses, 90, 15, 'curve')  # Adjust PWM for curve
    move_distance(int(straight_final), 80, 80, 'second_straight')  # Move straight for the final part

    rospy.loginfo(f"Pulses count: First straight - Right {pulses_first_straight[0]}, Left {pulses_first_straight[1]}, "
                  f"Curve - Right {pulses_curve[0]}, Left {pulses_curve[1]}, "
                  f"Second straight - Right {pulses_second_straight[0]}, Left {pulses_second_straight[1]}")

if __name__ == '__main__':
    try:
        start_l_turn_movement()
    finally:
        pwm_right.stop()
        pwm_left.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
