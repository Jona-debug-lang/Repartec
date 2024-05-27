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
def turn_90_degrees(pulses_turn,pwm_left_speed, pwm_right_speed):
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
    #pulsos_deseados_der = 6120  # Pulsos para 90 grados a la derecha
    
    print(f"Inicio de la curva: Pulsos deseados izq: {pulsos_deseados_izq}")
    #, der: {pulsos_deseados_der}
    
    while encoder_left_count < pulsos_deseados_izq:
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
    distancia_cm = 60
    move_straight(distancia_cm, 100, 100, eje='y')
    
    # Primera curva de 90 grados
    turn_90_degrees(6200,100, 0)
    
    # Segunda parte recta (30 cm) - Corrección en eje X
    distancia_cm = 20
    move_straight(distancia_cm, 100, 100, eje='x')
    
    # Segunda curva de 90 grados
    turn_90_degrees(7550,100, 0)

    # Tercera parte recta (110 cm) - Corrección en eje Y
    distancia_cm = 45
    move_straight(distancia_cm, 100, 100, eje='y')


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

[INFO] [1716003613.966383]: Lidar data collected: [(0.002, 0.006, 0.0), (-0.001, 0.005, -0.0), (0.002, 0.005, 0.01), (-0.028, 0.002, 0.0), (-0.061, -0.001, 0.0), (-0.097, -0.004, 0.0), (-0.126, -0.008, -0.0), (-0.162, -0.011, 0.0), (-0.194, -0.014, 0.01), (-0.22, -0.015, 0.0), (-0.251, -0.021, 0.01), (-0.287, -0.025, 0.01), (-0.317, -0.03, 0.01), (-0.345, -0.032, 0.01), (-0.377, -0.034, 0.0), (-0.408, -0.036, 0.0), (-0.451, -0.041, 0.0), (-0.489, -0.044, 0.0), (-0.528, -0.05, 0.0), (-0.567, -0.054, 0.0), (-0.604, -0.058, -0.0), (-0.63, -0.06, 0.0), (-0.649, -0.062, -0.0), (-0.681, -0.065, -0.01), (-0.698, -0.068, -0.02), (-0.725, -0.066, -0.04), (-0.743, -0.066, -0.05), (-0.773, -0.064, -0.08), (-0.8, -0.062, -0.1), (-0.83, -0.06, -0.12), (-0.854, -0.061, -0.13), (-0.872, -0.057, -0.14), (-0.894, -0.054, -0.16), (-0.909, -0.051, -0.17), (-0.933, -0.048, -0.2), (-0.944, -0.043, -0.21), (-0.969, -0.035, -0.23), (-1.0, -0.025, -0.25), (-1.026, -0.016, -0.26), (-1.066, -0.012, -0.28), (-1.095, -0.005, -0.3), (-1.126, 0.005, -0.32), (-1.162, 0.017, -0.35), (-1.197, 0.031, -0.38), (-1.219, 0.04, -0.4), (-1.24, 0.05, -0.42), (-1.278, 0.068, -0.45), (-1.309, 0.082, -0.48), (-1.338, 0.1, -0.51), (-1.366, 0.114, -0.54), (-1.399, 0.131, -0.57), (-1.427, 0.148, -0.59), (-1.454, 0.164, -0.63), (-1.477, 0.18, -0.67), (-1.497, 0.195, -0.69), (-1.517, 0.213, -0.71), (-1.539, 0.231, -0.73), (-1.564, 0.256, -0.76), (-1.586, 0.282, -0.78), (-1.606, 0.303, -0.8), (-1.626, 0.322, -0.82), (-1.642, 0.339, -0.83), (-1.659, 0.357, -0.84), (-1.672, 0.371, -0.85), (-1.693, 0.392, -0.86), (-1.709, 0.408, -0.87), (-1.723, 0.424, -0.89), (-1.74, 0.441, -0.91), (-1.752, 0.46, -0.92), (-1.766, 0.477, -0.93), (-1.781, 0.512, -0.99), (-1.79, 0.554, -1.04), (-1.818, 0.585, -1.08), (-1.836, 0.608, -1.1), (-1.853, 0.634, -1.11), (-1.866, 0.655, -1.13), (-1.877, 0.671, -1.14), (-1.89, 0.694, -1.17), (-1.896, 0.714, -1.19), (-1.905, 0.738, -1.22), (-1.91, 0.761, -1.25), (-1.915, 0.807, -1.3), (-1.918, 0.837, -1.32), (-1.924, 0.872, -1.35), (-1.933, 0.903, -1.36), (-1.939, 0.931, -1.38), (-1.948, 0.96, -1.4), (-1.954, 0.992, -1.43), (-1.96, 1.018, -1.46), (-1.963, 1.044, -1.47), (-1.965, 1.07, -1.48), (-1.97, 1.091, -1.49), (-1.975, 1.108, -1.5), (-1.978, 1.128, -1.5), (-1.977, 1.142, -1.51), (-1.98, 1.164, -1.51), (-1.982, 1.182, -1.51), (-1.987, 1.206, -1.51), (-1.997, 1.244, -1.51), (-2.003, 1.281, -1.52), (-2.017, 1.321, -1.53), (-2.023, 1.373, -1.54), (-2.021, 1.407, -1.57), (-2.021, 1.429, -1.58), (-2.021, 1.452, -1.6), (-2.02, 1.487, -1.62), (-2.018, 1.518, -1.63), (-2.016, 1.55, -1.65), (-2.012, 1.583, -1.68), (-2.007, 1.627, -1.71), (-2.004, 1.666, -1.74), (-2.001, 1.691, -1.74), (-1.993, 1.721, -1.76), (-1.991, 1.75, -1.78), (-1.988, 1.778, -1.79), (-1.982, 1.812, -1.81), (-1.97, 1.856, -1.83), (-1.962, 1.883, -1.84), (-1.949, 1.911, -1.86), (-1.938, 1.944, -1.9), (-1.931, 1.974, -1.91), (-1.923, 2.002, -1.93), (-1.913, 2.028, -1.94), (-1.904, 2.052, -1.97), (-1.897, 2.069, -1.97), (-1.889, 2.085, -1.98), (-1.881, 2.106, -2.0), (-1.869, 2.125, -2.01), (-1.854, 2.158, -2.04), (-1.843, 2.177, -2.06), (-1.834, 2.203, -2.08), (-1.822, 2.218, -2.1), (-1.811, 2.249, -2.13), (-1.795, 2.28, -2.15), (-1.783, 2.302, -2.15), (-1.77, 2.321, -2.17), (-1.759, 2.338, -2.18), (-1.749, 2.354, -2.18), (-1.734, 2.374, -2.2), (-1.717, 2.399, -2.22), (-1.706, 2.415, -2.23), (-1.684, 2.441, -2.25), (-1.67, 2.454, -2.27), (-1.654, 2.477, -2.3), (-1.642, 2.488, -2.31), (-1.629, 2.502, -2.33), (-1.611, 2.52, -2.35), (-1.595, 2.539, -2.36), (-1.582, 2.554, -2.37), (-1.565, 2.57, -2.38), (-1.551, 2.582, -2.39), (-1.537, 2.596, -2.41), (-1.515, 2.615, -2.42), (-1.492, 2.634, -2.44), (-1.47, 2.648, -2.46), (-1.452, 2.662, -2.48), (-1.433, 2.675, -2.51), (-1.412, 2.687, -2.51), (-1.394, 2.704, -2.54), (-1.374, 2.717, -2.55), (-1.333, 2.744, -2.6), (-1.293, 2.768, -2.62), (-1.263, 2.787, -2.63), (-1.24, 2.798, -2.64), (-1.211, 2.817, -2.67), (-1.191, 2.826, -2.68), (-1.158, 2.842, -2.71), (-1.126, 2.857, -2.74), (-1.097, 2.87, -2.76), (-1.059, 2.881, -2.79), (-1.025, 2.892, -2.82), (-0.987, 2.907, -2.85), (-0.955, 2.919, -2.87), (-0.929, 2.926, -2.88), (-0.901, 2.935, -2.9), (-0.873, 2.942, -2.92), (-0.838, 2.949, -2.95), (-0.806, 2.955, -2.97), (-0.773, 2.961, -2.99), (-0.738, 2.965, -3.03), (-0.714, 2.97, -3.04), (-0.69, 2.974, -3.05), (-0.663, 2.979, -3.06), (-0.63, 2.982, -3.09), (-0.6, 2.984, -3.1), (-0.565, 2.989, -3.11), (-0.529, 2.995, -3.12), (-0.458, 3.005, -3.12), (-0.403, 3.009, -3.12), (-0.35, 3.016, -3.13), (-0.279, 3.023, -3.14), (-0.235, 3.024, 3.14)]
[INFO] [1716003613.971222]: Theta changes during turns: [-1.46, -1.5399999999999998]
ubuntu@ubuntu:~/catkin_ws/src/my_python_scripts/scripts$ 
