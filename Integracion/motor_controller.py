#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import math
import numpy as np

# Configuración de GPIO para motores y encoders
PIN_MOTOR_A = 17
PIN_MOTOR_B = 27
ENCODER_A_PIN_A = 22
ENCODER_A_PIN_B = 23
ENCODER_B_PIN_A = 24
ENCODER_B_PIN_B = 25

# Desactivar advertencias de GPIO si no son útiles
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()  # Limpiar todos los pines GPIO

# Configurar pines de motores como salidas y pines de encoders como entradas
GPIO.setup([PIN_MOTOR_A, PIN_MOTOR_B], GPIO.OUT)
GPIO.setup([ENCODER_A_PIN_A, ENCODER_A_PIN_B, ENCODER_B_PIN_A, ENCODER_B_PIN_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Configuración de PWM
pwm_a = GPIO.PWM(PIN_MOTOR_A, 1000)
pwm_b = GPIO.PWM(PIN_MOTOR_B, 1000)
pwm_a.start(0)
pwm_b.start(0)

# Inicializar nodo ROS
rospy.init_node('motor_controller', anonymous=True)

# Variables globales para el PID y encoder
integral = 0
last_error = 0
Kp = 0.5
Ki = 0.1
Kd = 0.05
dt = 0.1
encoder_count_a = 0
encoder_count_b = 0

# Variables globales para las metas
goal_x = 0.0  # Valor inicial por defecto
goal_y = 0.0  # Valor inicial por defecto

# Estado y matrices del EKF
x_est = np.zeros(5)  # [x, y, theta, v, omega]
P = np.eye(5) * 0.01  # Matriz de covarianza inicial pequeña
Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.01])  # Ruido de proceso
R = np.diag([0.5, 0.5, 0.5, 0.1, 0.1])  # Ruido de medición

# Funciones de callback para encoders
def encoder_callback_a(channel):
    global encoder_count_a
    phase_a = GPIO.input(ENCODER_A_PIN_A)
    phase_b = GPIO.input(ENCODER_A_PIN_B)
    if phase_a == phase_b:
        encoder_count_a += 1
    else:
        encoder_count_a -= 1

def encoder_callback_b(channel):
    global encoder_count_b
    phase_a = GPIO.input(ENCODER_B_PIN_A)
    phase_b = GPIO.input(ENCODER_B_PIN_B)
    if phase_a == phase_b:
        encoder_count_b += 1
    else:
        encoder_count_b -= 1

# Añadir detección de eventos en los pines de encoders
GPIO.add_event_detect(ENCODER_A_PIN_A, GPIO.BOTH, callback=encoder_callback_a)
GPIO.add_event_detect(ENCODER_B_PIN_A, GPIO.BOTH, callback=encoder_callback_b)

def handle_goal(data):
    global goal_x, goal_y
    goal_x = data.data[0]
    goal_y = data.data[1]
    rospy.loginfo(f"Objetivo actualizado x: {goal_x}, y: {goal_y}")

def pid_calculate(error):
    global integral, last_error, Kp, Ki, Kd, dt
    integral += error * dt
    derivative = (error - last_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    return output

def handle_pose(data):
    global x_est, current_time
    new_time = rospy.get_time()
    dt = new_time - current_time
    current_time = new_time
    z = [data.x, data.y, data.theta, encoder_count_a * 0.05 * dt, encoder_count_b * 0.05 * dt]
    ekf_update(z, dt)

def ekf_update(z, dt):
    global x_est, P, Q, R
    theta = x_est[2]
    F = np.array([
        [1, 0, -x_est[3] * dt * np.sin(theta), dt * np.cos(theta), 0],
        [0, 1, x_est[3] * dt * np.cos(theta), dt * np.sin(theta), 0],
        [0, 0, 1, 0, dt],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])
    x_est = F @ x_est
    P = F @ P @ F.T + Q
    H = np.eye(5)
    y = np.array(z) - H @ x_est
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_est += K @ y
    P = (np.eye(len(x_est)) - K @ H) @ P

def control_motors():
    global x_est, pwm_a, pwm_b, goal_x, goal_y
    x, y, theta, v, omega = x_est
    angle_to_goal = math.atan2(goal_y - y, goal_x - x)
    angle_error = (angle_to_goal - theta + math.pi) % (2 * math.pi) - math.pi
    angular_speed_correction = pid_calculate(angle_error)
    base_speed = 50
    speed_a = max(0, min(100, base_speed + angular_speed_correction))
    speed_b = max(0, min(100, base_speed - angular_speed_correction))
    pwm_a.ChangeDutyCycle(speed_a)
    pwm_b.ChangeDutyCycle(speed_b)
    rospy.loginfo(f"Motor speeds set to A: {speed_a}%, B: {speed_b}% based on angle error: {angle_error}")

if __name__ == '__main__':
    rospy.Subscriber("/simple_pose", Pose2D, handle_pose)
    rospy.Subscriber("/coordenadas_goal", Float32MultiArray, handle_goal)
    r = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        control_motors()
        r.sleep()
