#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from obstacle_detector.msg import Obstacles  # Asumiendo que estás usando el mensaje Obstacles del paquete obstacle_detector
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

# Variables para la corrección de ruta y evasión de obstáculos
deviation_y = 0
deviation_x = 0
lidar_data = []
theta_changes = []
should_stop = False
obstacle_detected = False
evasion_active = False

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

# Suscripción a los obstáculos
def obstacles_callback(data):
    global should_stop, obstacle_detected, evasion_active
    if evasion_active:
        return  # No hacer nada si ya estamos evadiendo un obstáculo

    obstacle_detected = False
    for circle in data.circles:
        if circle.velocity.x ** 2 + circle.velocity.y ** 2 > 0.1 ** 2:
            should_stop = True
            return
        # Definir la zona de seguridad
        if -0.14 < circle.center.y < 0.14 and 0 < circle.center.x < 0.1:  # 14 cm a la derecha, 14 cm a la izquierda, y 10 cm al frente
            obstacle_detected = True
            rospy.loginfo("Obstáculo detectado en la zona de seguridad")
            return
    should_stop = False

rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

# Publicador para el mensaje "finish_move"
done_pub = rospy.Publisher('finish_move', String, queue_size=10)

# Función para mover los motores en línea recta con corrección de ruta y evasión de obstáculos
def move_straight(distancia_cm, pwm_left_speed, pwm_right_speed, eje):
    global start_time, end_time, encoder_left_count, encoder_right_count, deviation_y, deviation_x, should_stop, obstacle_detected
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
    
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        adjust_movement(deviation_y if eje == 'y' else deviation_x, eje)  # Llamada correcta a la función
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

# Función para la evasión de obstáculos
def evade_obstacle():
    global encoder_left_count, encoder_right_count, evasion_active, should_stop
    evasion_active = True
    
    # Suprimir la corrección de ruta durante la evasión
    correct_route = False
    
    # Definir la dirección de la evasión (izquierda o derecha)
    evade_direction = "left" if deviation_y > 0 else "right"

    # Evasión basada en el radio de evasión especificado
    evade_radius = 0.2151915266710571  # Radio de evasión en cm

    initial_encoder_left_count = encoder_left_count
    initial_encoder_right_count = encoder_right_count

    pulses_evade_left = evade_radius * k_izq
    pulses_evade_right = evade_radius * k_der

    # Ajustar PWM para la curva de evasión
    if evade_direction == "left":
        pwm_left.ChangeDutyCycle(70)
        pwm_right.ChangeDutyCycle(100)
    else:
        pwm_left.ChangeDutyCycle(100)
        pwm_right.ChangeDutyCycle(70)
    
    rospy.loginfo(f"Evadiendo obstáculo hacia la {evade_direction}")

    while (encoder_left_count - initial_encoder_left_count) < pulses_evade_left and (encoder_right_count - initial_encoder_right_count) < pulses_evade_right:
        time.sleep(0.01)

    # Segunda fase: Regreso a la trayectoria original
    if evade_direction == "left":
        pwm_left.ChangeDutyCycle(100)
        pwm_right.ChangeDutyCycle(70)
    else:
        pwm_left.ChangeDutyCycle(70)
        pwm_right.ChangeDutyCycle(100)
    
    initial_encoder_left_count = encoder_left_count
    initial_encoder_right_count = encoder_right_count

    while (encoder_left_count - initial_encoder_left_count) < pulses_evade_left and (encoder_right_count - initial_encoder_right_count) < pulses_evade_right:
        time.sleep(0.01)

    # Restaurar los PWM a sus valores originales
    pwm_left.ChangeDutyCycle(100)
    pwm_right.ChangeDutyCycle(100)

    # Reactivar la corrección de ruta
    correct_route = True
    evasion_active = False

    rospy.loginfo("Evasión completada")

# Función para girar 90 grados
def turn_90_degrees(pulses_turn, pwm_left_speed, pwm_right_speed):
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
    
    rospy.loginfo(f"Inicio de la curva: Pulsos deseados izq: {pulsos_deseados_izq}")
    
    while encoder_left_count < pulsos_deseados_izq:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    
    # Obtener theta final del LIDAR
    final_theta = lidar_data[-1][2] if lidar_data else 0
    theta_change = final_theta - initial_theta
    theta_changes.append(theta_change)
    
    rospy.loginfo("Motores detenidos")

# Función para girar 90 grados en su propio eje
def turn_in_place_90_degrees(pulses_turn, pwm_speed):
    global start_time, end_time, encoder_left_count, encoder_right_count
    initial_theta = lidar_data[-1][2] if lidar_data else 0  # Obtener theta inicial del LIDAR
    GPIO.output(MOTOR_IZQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_speed)
    pwm_right.ChangeDutyCycle(pwm_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados = pulses_turn  # Ajusta este valor según tus necesidades
    
    rospy.loginfo(f"Inicio de la curva en su lugar: Pulsos deseados: {pulsos_deseados}")
    
    while encoder_left_count < pulsos_deseados:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm.right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(pwm_speed)
            pwm_right.ChangeDutyCycle(pwm_speed)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    
    # Obtener theta final del LIDAR
    final_theta = lidar_data[-1][2] if lidar_data else 0
    theta_change = final_theta - initial_theta
    theta_changes.append(theta_change)
    
    rospy.loginfo("Motores detenidos")

# Función para ajustar el movimiento basado en la desviación del Lidar
def adjust_movement(deviation, eje):
    if evasion_active:
        return  # No ajustar el movimiento si estamos evadiendo un obstáculo

    if eje == 'y':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(80)  # Ajustar duty cycle para corrección
            pwm_right.ChangeDutyCycle(100)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(85)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(100)
    elif eje == 'x':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(80)
            pwm_right.ChangeDutyCycle(100)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(85)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(100)
    rospy.loginfo(f"Adjusting due to {eje} deviation: {deviation:.3f}")

# Función para manejar el comando T1
def handle_T1():
    # Primera parte recta (60 cm) - Corrección en eje Y
    distancia_cm = 60
    move_straight(distancia_cm, 100, 100, 'y')  # Velocidades calibradas
    
    # Primera curva de 90 grados
    turn_90_degrees(6200, 100, 0)
    
    # Segunda parte recta (20 cm) - Corrección en eje X
    distancia_cm = 20
    move_straight(distancia_cm, 100, 100, 'x')  # Velocidades calibradas
    
    # Segunda curva de 90 grados
    turn_90_degrees(7550, 100, 0)

    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    
    # Tercera parte recta (45 cm) - Corrección en eje Y
    distancia_cm = 45
    move_straight(distancia_cm, 100, 100, 'y')  # Velocidades calibradas

    # Salida a terreno ideal
    distancia_cm = 60
    move_straight(distancia_cm, 80, 80, 'y')  # Velocidad reducida

    # Vuelta a 90 grados en su propio eje en terreno ideal
    turn_in_place_90_degrees(3600, 70)  # Velocidad reducida

    # Recta en terreno ideal
    distancia_cm = 200
    move_straight(distancia_cm, 90, 90, 'x')  # Velocidad reducida

    # Segunda vuelta de 90 grados en su propio eje y final
    turn_in_place_90_degrees(3600, 70)  # Velocidad reducida

    # Regreso a terreno anterior
    distancia_cm = 60
    move_straight(distancia_cm, 68, 68, 'y')  # Velocidad reducida

# Función para manejar el comando T2
def handle_T2():
    # Mover 200 cm hacia adelante
    distancia_cm = 200
    move_straight(distancia_cm, 100, 100, 'y')  # Velocidades calibradas
    
    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    
    # Mover 200 cm en reversa
    GPIO.output(MOTOR_IZQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(100)
    pwm_right.ChangeDutyCycle(100)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento reversa: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(100)
            pwm.right.ChangeDutyCycle(100)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    if data.data == "T1":
        handle_T1()
    elif data.data == "T2":
        handle_T2()
    done_pub.publish("Done")
    rospy.loginfo("Mensaje 'Done' publicado")

# Nodo para escuchar comandos
rospy.Subscriber('first_move', String, command_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
        rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
        rospy.loginfo(f"Lidar data collected: {lidar_data}")
        rospy.loginfo(f"Theta changes during turns: {theta_changes}")
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from obstacle_detector.msg import Obstacles  # Asumiendo que estás usando el mensaje Obstacles del paquete obstacle_detector
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

# Variables para la corrección de ruta y evasión de obstáculos
deviation_y = 0
deviation_x = 0
lidar_data = []
theta_changes = []
should_stop = False
obstacle_detected = False
evasion_active = False

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

# Suscripción a los obstáculos
def obstacles_callback(data):
    global should_stop, obstacle_detected, evasion_active
    if evasion_active:
        return  # No hacer nada si ya estamos evadiendo un obstáculo

    obstacle_detected = False
    for circle in data.circles:
        if circle.velocity.x ** 2 + circle.velocity.y ** 2 > 0.1 ** 2:
            should_stop = True
            return
        # Definir la zona de seguridad
        if -0.14 < circle.center.y < 0.14 and 0 < circle.center.x < 0.1:  # 14 cm a la derecha, 14 cm a la izquierda, y 10 cm al frente
            obstacle_detected = True
            rospy.loginfo("Obstáculo detectado en la zona de seguridad")
            return
    should_stop = False

rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

# Publicador para el mensaje "finish_move"
done_pub = rospy.Publisher('finish_move', String, queue_size=10)

# Función para mover los motores en línea recta con corrección de ruta y evasión de obstáculos
def move_straight(distancia_cm, pwm_left_speed, pwm_right_speed, eje):
    global start_time, end_time, encoder_left_count, encoder_right_count, deviation_y, deviation_x, should_stop, obstacle_detected
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
    
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        adjust_movement(deviation_y if eje == 'y' else deviation_x, eje)  # Llamada correcta a la función
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

# Función para la evasión de obstáculos
def evade_obstacle():
    global encoder_left_count, encoder_right_count, evasion_active, should_stop
    evasion_active = True
    
    # Suprimir la corrección de ruta durante la evasión
    correct_route = False
    
    # Definir la dirección de la evasión (izquierda o derecha)
    evade_direction = "left" if deviation_y > 0 else "right"

    # Evasión basada en el radio de evasión especificado
    evade_radius = 0.2151915266710571  # Radio de evasión en cm

    initial_encoder_left_count = encoder_left_count
    initial_encoder_right_count = encoder_right_count

    pulses_evade_left = evade_radius * k_izq
    pulses_evade_right = evade_radius * k_der

    # Ajustar PWM para la curva de evasión
    if evade_direction == "left":
        pwm_left.ChangeDutyCycle(70)
        pwm_right.ChangeDutyCycle(100)
    else:
        pwm_left.ChangeDutyCycle(100)
        pwm_right.ChangeDutyCycle(70)
    
    rospy.loginfo(f"Evadiendo obstáculo hacia la {evade_direction}")

    while (encoder_left_count - initial_encoder_left_count) < pulses_evade_left and (encoder_right_count - initial_encoder_right_count) < pulses_evade_right:
        time.sleep(0.01)

    # Segunda fase: Regreso a la trayectoria original
    if evade_direction == "left":
        pwm_left.ChangeDutyCycle(100)
        pwm_right.ChangeDutyCycle(70)
    else:
        pwm_left.ChangeDutyCycle(70)
        pwm_right.ChangeDutyCycle(100)
    
    initial_encoder_left_count = encoder_left_count
    initial_encoder_right_count = encoder_right_count

    while (encoder_left_count - initial_encoder_left_count) < pulses_evade_left and (encoder_right_count - initial_encoder_right_count) < pulses_evade_right:
        time.sleep(0.01)

    # Restaurar los PWM a sus valores originales
    pwm_left.ChangeDutyCycle(100)
    pwm_right.ChangeDutyCycle(100)

    # Reactivar la corrección de ruta
    correct_route = True
    evasion_active = False

    rospy.loginfo("Evasión completada")

# Función para girar 90 grados
def turn_90_degrees(pulses_turn, pwm_left_speed, pwm_right_speed):
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
    
    rospy.loginfo(f"Inicio de la curva: Pulsos deseados izq: {pulsos_deseados_izq}")
    
    while encoder_left_count < pulsos_deseados_izq:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    
    # Obtener theta final del LIDAR
    final_theta = lidar_data[-1][2] if lidar_data else 0
    theta_change = final_theta - initial_theta
    theta_changes.append(theta_change)
    
    rospy.loginfo("Motores detenidos")

# Función para girar 90 grados en su propio eje
def turn_in_place_90_degrees(pulses_turn, pwm_speed):
    global start_time, end_time, encoder_left_count, encoder_right_count
    initial_theta = lidar_data[-1][2] if lidar_data else 0  # Obtener theta inicial del LIDAR
    GPIO.output(MOTOR_IZQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_speed)
    pwm_right.ChangeDutyCycle(pwm_speed)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados = pulses_turn  # Ajusta este valor según tus necesidades
    
    rospy.loginfo(f"Inicio de la curva en su lugar: Pulsos deseados: {pulsos_deseados}")
    
    while encoder_left_count < pulsos_deseados:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm.right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(pwm_speed)
            pwm_right.ChangeDutyCycle(pwm_speed)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    
    # Obtener theta final del LIDAR
    final_theta = lidar_data[-1][2] if lidar_data else 0
    theta_change = final_theta - initial_theta
    theta_changes.append(theta_change)
    
    rospy.loginfo("Motores detenidos")

# Función para ajustar el movimiento basado en la desviación del Lidar
def adjust_movement(deviation, eje):
    if evasion_active:
        return  # No ajustar el movimiento si estamos evadiendo un obstáculo

    if eje == 'y':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(80)  # Ajustar duty cycle para corrección
            pwm_right.ChangeDutyCycle(100)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(85)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(100)
    elif eje == 'x':
        if deviation > 0.01:  # Desviación positiva mayor a 1 cm
            pwm_left.ChangeDutyCycle(80)
            pwm_right.ChangeDutyCycle(100)
        elif deviation < -0.01:  # Desviación negativa mayor a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(85)
        else:
            # Restablecer los PWM a valores normales si la desviación es menor o igual a 1 cm
            pwm_left.ChangeDutyCycle(100)
            pwm_right.ChangeDutyCycle(100)
    rospy.loginfo(f"Adjusting due to {eje} deviation: {deviation:.3f}")

# Función para manejar el comando T1
def handle_T1():
    # Primera parte recta (60 cm) - Corrección en eje Y
    distancia_cm = 60
    move_straight(distancia_cm, 100, 100, 'y')  # Velocidades calibradas
    
    # Primera curva de 90 grados
    turn_90_degrees(6200, 100, 0)
    
    # Segunda parte recta (20 cm) - Corrección en eje X
    distancia_cm = 20
    move_straight(distancia_cm, 100, 100, 'x')  # Velocidades calibradas
    
    # Segunda curva de 90 grados
    turn_90_degrees(7550, 100, 0)

    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    
    # Tercera parte recta (45 cm) - Corrección en eje Y
    distancia_cm = 45
    move_straight(distancia_cm, 100, 100, 'y')  # Velocidades calibradas

    # Salida a terreno ideal
    distancia_cm = 60
    move_straight(distancia_cm, 80, 80, 'y')  # Velocidad reducida

    # Vuelta a 90 grados en su propio eje en terreno ideal
    turn_in_place_90_degrees(3600, 70)  # Velocidad reducida

    # Recta en terreno ideal
    distancia_cm = 200
    move_straight(distancia_cm, 90, 90, 'x')  # Velocidad reducida

    # Segunda vuelta de 90 grados en su propio eje y final
    turn_in_place_90_degrees(3600, 70)  # Velocidad reducida

    # Regreso a terreno anterior
    distancia_cm = 60
    move_straight(distancia_cm, 68, 68, 'y')  # Velocidad reducida

# Función para manejar el comando T2
def handle_T2():
    # Mover 200 cm hacia adelante
    distancia_cm = 200
    move_straight(distancia_cm, 100, 100, 'y')  # Velocidades calibradas
    
    rospy.loginfo("Esperando mensaje continue_move...")
    rospy.wait_for_message('continue_move', String)
    
    # Mover 200 cm en reversa
    GPIO.output(MOTOR_IZQ_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN3, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(100)
    pwm_right.ChangeDutyCycle(100)
    
    encoder_left_count = 0
    encoder_right_count = 0
    start_time = time.time()
    
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    rospy.loginfo(f"Inicio del movimiento reversa: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while encoder_left_count < pulsos_deseados_izq or encoder_right_count < pulsos_deseados_der:
        rospy.loginfo(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        if should_stop:
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            rospy.loginfo("Movimiento pausado por detección de objeto en movimiento")
            while should_stop:
                time.sleep(0.1)
            pwm_left.ChangeDutyCycle(100)
            pwm.right.ChangeDutyCycle(100)
            rospy.loginfo("Movimiento reanudado")
        elif obstacle_detected:
            rospy.loginfo("Evitando obstáculo")
            evade_obstacle()  # Llamada a la función de evasión
            rospy.loginfo("Obstáculo evitado, reanudando movimiento")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    pwm_left.ChangeDutyCycle(0)  # Detener los motores
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Motores detenidos")

def command_callback(data):
    rospy.loginfo(f"Comando recibido: {data.data}")
    if data.data == "T1":
        handle_T1()
    elif data.data == "T2":
        handle_T2()
    done_pub.publish("Done")
    rospy.loginfo("Mensaje 'Done' publicado")

# Nodo para escuchar comandos
rospy.Subscriber('first_move', String, command_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
        rospy.signal_shutdown("Finished movement")
        rospy.loginfo(f"Final counts: Left {encoder_left_count}, Right {encoder_right_count}")
        rospy.loginfo(f"Lidar data collected: {lidar_data}")
        rospy.loginfo(f"Theta changes during turns: {theta_changes}")
