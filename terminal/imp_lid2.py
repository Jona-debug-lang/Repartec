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
pwm_right = GPIO.PWM(ENABLE2, 1000)  # ENABLE2 corresponde al motor derecho
pwm_left = GPIO.PWM(ENABLE1, 1000)   # ENABLE1 corresponde al motor izquierdo
pwm_right.start(100)  # Iniciar con un duty cycle adecuado
pwm_left.start(100)

# Variables para el seguimiento de encoders
encoder_right_count = 0
encoder_left_count = 0
encoder_right_last_state = GPIO.input(ENCODER_RIGHT_A)
encoder_left_last_state = GPIO.input(ENCODER_LEFT_A)

# Función de callback para los encoders
def encoder_callback_right(channel):
    global encoder_right_count, encoder_right_last_state
    current_state = GPIO.input(ENCODER_RIGHT_A)
    if current_state != encoder_right_last_state:
        if GPIO.input(ENCODER_RIGHT_B) != current_state:
            encoder_right_count += 1
        else:
            encoder_right_count -= 1
        encoder_right_last_state = current_state
    rospy.loginfo(f"Encoder Right count: {encoder_right_count}")

def encoder_callback_left(channel):
    global encoder_left_count, encoder_left_last_state
    current_state = GPIO.input(ENCODER_LEFT_A)
    if current_state != encoder_left_last_state:
        if GPIO.input(ENCODER_LEFT_B) != current_state:
            encoder_left_count += 1
        else:
            encoder_left_count -= 1
        encoder_left_last_state = current_state
    rospy.loginfo(f"Encoder Left count: {encoder_left_count}")

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.BOTH, callback=encoder_callback_right)
GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.BOTH, callback=encoder_callback_left)

# Suscripción a la posición del Lidar
def pose_callback(data):
    rospy.loginfo("Received Lidar data: x={:.3f}, y={:.3f}, theta={:.2f}".format(data.x, data.y, data.theta))
    if abs(data.y) > 0.01:  # Más de 1 cm de desviación en Y
        adjust_movement(data.y)
    else:
        # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
        pwm_right.ChangeDutyCycle(100)
        pwm_left.ChangeDutyCycle(100)

rospy.Subscriber('simple_pose', Pose2D, pose_callback)

def adjust_movement(y_deviation):
    if y_deviation > 0.01:  # Desviación positiva mayor a 1 cm
        pwm_right.ChangeDutyCycle(90)  # Ajustar duty cycle para corrección
        pwm_left.ChangeDutyCycle(95)
    elif y_deviation < -0.01:  # Desviación negativa mayor a 1 cm
        pwm_right.ChangeDutyCycle(95)
        pwm_left.ChangeDutyCycle(90)
    rospy.loginfo(f"Adjusting due to y deviation: {y_deviation:.3f}")

def get_pulses_per_cm(pwm_value):
    # Definir una tabla de calibración simple basada en el ciclo de trabajo del PWM
    if pwm_value >= 90:
        return 2.58  # Valor para PWM >= 90
    elif pwm_value >= 80:
        return 2.65  # Ajustar según sea necesario
    elif pwm_value >= 70:
        return 2.75  # Ajustar según sea necesario
    # Agregar más entradas según sea necesario
    else:
        return 2.85  # Valor por defecto o el más bajo

def move_straight(target_distance_cm, pwm_value=100):
    pulses_per_cm = get_pulses_per_cm(pwm_value)
    target_pulses = int(target_distance_cm * pulses_per_cm)
    initial_pulses = (encoder_right_count + encoder_left_count) // 2

    rospy.loginfo(f"Starting straight movement with PWM: {pwm_value}%")
    GPIO.output([IN1, IN3], GPIO.HIGH)
    GPIO.output([IN2, IN4], GPIO.LOW)
    pwm_right.ChangeDutyCycle(pwm_value)
    pwm_left.ChangeDutyCycle(pwm_value)

    while True:
        current_pulses = (encoder_right_count + encoder_left_count) // 2
        rospy.loginfo(f"Current Pulses: {current_pulses}, Target Pulses: {target_pulses}, Right: {encoder_right_count}, Left: {encoder_left_count}")
        if current_pulses - initial_pulses >= target_pulses:
            break
        rospy.sleep(0.01)

    # Stop motors
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwm_right.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)
    rospy.loginfo("Movement completed.")

if __name__ == '__main__':
    try:
        move_straight(200, 100)  # Mover 200 cm con PWM del 100%
    finally:
        pwm_right.stop()
        pwm_left.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Movement finished")
