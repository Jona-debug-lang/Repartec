#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
import RPi.GPIO as GPIO
import time

# Inicializar nodo ROS
rospy.init_node('evade_obstacle', anonymous=True, log_level=rospy.DEBUG)

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

# Variables para detección de obstáculos
should_stop = False
obstacle_position = None

# Rango de detección de obstáculos (en metros)
detection_range_min = 0.15
detection_range_max = 0.20

# Función de callback para los encoders
def encoder_callback_izq(channel):
    global encoder_left_count
    encoder_left_count += 1
    rospy.logdebug(f"Encoder izquierdo: {encoder_left_count}")

def encoder_callback_der(channel):
    global encoder_right_count
    encoder_right_count += 1
    rospy.logdebug(f"Encoder derecho: {encoder_right_count}")

# Configura las interrupciones de los encoders
GPIO.add_event_detect(ENCODER_IZQ, GPIO.RISING, callback=encoder_callback_izq)
GPIO.add_event_detect(ENCODER_DER, GPIO.RISING, callback=encoder_callback_der)

# Función de callback para la detección de obstáculos
def obstacles_callback(data):
    global should_stop, obstacle_position
    for circle in data.circles:
        distance_to_obstacle = (circle.center.x ** 2 + circle.center.y ** 2) ** 0.5
        rospy.logdebug(f"Datos del círculo: posición=({circle.center.x}, {circle.center.y}), radio={circle.radius}, distancia={distance_to_obstacle}")
        if detection_range_min <= distance_to_obstacle <= detection_range_max:  # Ajustar según el tamaño y posición del termo
            should_stop = True
            obstacle_position = (circle.center.x, circle.center.y)
            rospy.loginfo(f"Obstáculo detectado dentro del rango: posición=({circle.center.x}, {circle.center.y}), distancia={distance_to_obstacle}")
            return

# Función para suscribirse al tópico de obstáculos
def subscribe_to_obstacles():
    rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

# Función para mover el robot y verificar obstáculos
def move_straight_distance(distance_cm, pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count, should_stop, obstacle_position

    # Inicializar los encoders y las PWM
    encoder_left_count = 0
    encoder_right_count = 0
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)

    # Depuración
    rospy.logdebug("Motores activados: IN1=HIGH, IN2=LOW, IN3=HIGH, IN4=LOW")
    rospy.logdebug(f"Duty Cycle: pwm_left={pwm_left_speed}, pwm_right={pwm_right_speed}")
    
    # Calcular los pulsos deseados
    pulsos_deseados_izq = distance_cm * k_izq
    pulsos_deseados_der = distance_cm * k_der
    rospy.loginfo(f"Inicio del movimiento recto: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")

    while encoder_left_count < pulsos_deseados_izq and encoder_right_count < pulsos_deseados_der:
        rospy.logdebug(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")

        if should_stop:
            rospy.loginfo("Obstáculo detectado, deteniendo para analizar")
            pwm_left.ChangeDutyCycle(0)
            pwm_right.ChangeDutyCycle(0)
            time.sleep(2)  # Pausa para analizar el obstáculo
            evade_obstacle()
            should_stop = False
            obstacle_position = None  # Reset obstacle position
            pwm_left.ChangeDutyCycle(pwm_left_speed)
            pwm_right.ChangeDutyCycle(pwm_right_speed)

        time.sleep(0.01)  # Ajusta este valor según sea necesario

    # Detener los motores
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Movimiento completado: Motores detenidos")

# Función para evadir el obstáculo con curvas suaves
def evade_obstacle():
    global obstacle_position
    if obstacle_position:
        rospy.loginfo(f"Evitando obstáculo en posición: {obstacle_position}")
        # Desactivar la suscripción para evitar ruido durante la evasión
        obstacle_sub.unregister()
        
        # Implementa la lógica para evadir el obstáculo con una curva suave
        if obstacle_position[1] > 0:  # Obstáculo a la derecha
            # Girar a la izquierda suavemente
            curve_left(1500, 40, 60)  # Ajustar pulsos y velocidad para curva más lenta
            move_straight_distance(20, 45, 50)  # Mover 20 cm hacia adelante
            curve_right(1500, 60, 40)  # Girar a la derecha suavemente para volver a la ruta
        else:  # Obstáculo a la izquierda
            # Girar a la derecha suavemente
            curve_right(1500, 60, 40)  # Ajustar pulsos y velocidad para curva más lenta
            move_straight_distance(20, 45, 50)  # Mover 20 cm hacia adelante
            curve_left(1500, 40, 60)  # Girar a la izquierda suavemente para volver a la ruta

        # Reactivar la suscripción después de la evasión
        subscribe_to_obstacles()

# Función para realizar una curva a la izquierda suavemente
def curve_left(pulses_turn, pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)

    encoder_left_count = 0
    encoder_right_count = 0
    rospy.loginfo(f"Iniciando curva a la izquierda: Pulsos deseados izq: {pulses_turn}")
    
    while encoder_left_count < pulses_turn and encoder_right_count < pulses_turn:
        rospy.logdebug(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario

    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Curva a la izquierda completada: Motores detenidos")

# Función para realizar una curva a la derecha suavemente
def curve_right(pulses_turn, pwm_left_speed, pwm_right_speed):
    global encoder_left_count, encoder_right_count
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(pwm_left_speed)
    pwm_right.ChangeDutyCycle(pwm_right_speed)

    encoder_left_count = 0
    encoder_right_count = 0
    rospy.loginfo(f"Iniciando curva a la derecha: Pulsos deseados izq: {pulses_turn}")
    
    while encoder_left_count < pulses_turn and encoder_right_count < pulses_turn:
        rospy.logdebug(f"Pulsos actuales: izq: {encoder_left_count}, der: {encoder_right_count}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario

    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    rospy.loginfo("Curva a la derecha completada: Motores detenidos")

# Suscribir al tópico de obstáculos al inicio
obstacle_sub = rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

if __name__ == '__main__':
    try:
        rospy.loginfo("Iniciando movimiento y evasión de obstáculos")
        move_straight_distance(200, 40, 40)  # Mover hacia adelante por 2 metros a una velocidad más lenta
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupción de ROS detectada. Apagando el nodo.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup and node shutdown.")
87), radio=0.1104982097120462, distancia=1.507658852165199
[DEBUG] [1716676554.803993]: Encoder derecho: 314
[DEBUG] [1716676554.817503]: Datos del círculo: posición=(-1.5015511820638927, -0.12874250964874842), radio=0.10858969761443192, distancia=1.5070602463565057
[DEBUG] [1716676554.822942]: Encoder izquierdo: 316
[DEBUG] [1716676554.824212]: Pulsos actuales: izq: 316, der: 314
[DEBUG] [1716676554.828434]: Encoder derecho: 315
[DEBUG] [1716676554.835944]: Encoder izquierdo: 317
[DEBUG] [1716676554.842319]: Datos del círculo: posición=(-1.5010442659274426, -0.12927490558527652), radio=0.10711785964718211, distancia=1.5066007730940991
[DEBUG] [1716676554.853159]: Encoder derecho: 316
[DEBUG] [1716676554.859281]: Pulsos actuales: izq: 317, der: 316
[DEBUG] [1716676554.863112]: Datos del círculo: posición=(-1.5006648905620141, -0.1296725566742243), radio=0.10601240415810582, distancia=1.5062569786460516
[DEBUG] [1716676554.875477]: Datos del círculo: posición=(-1.5003848821260386, -0.12996552827810842), radio=0.10519388531112275, distancia=1.5060032646222832
[DEBUG] [1716676554.888238]: Encoder izquierdo: 318
[DEBUG] [1716676554.892330]: Pulsos actuales: izq: 318, der: 316
[DEBUG] [1716676554.897289]: Encoder derecho: 317
[DEBUG] [1716676554.912770]: Datos del círculo: posición=(-1.5001798106307813, -0.13017970922959354), radio=0.10459251520928971, distancia=1.5058174593619602
[DEBUG] [1716676554.931301]: Pulsos actuales: izq: 318, der: 317
[DEBUG] [1716676554.932278]: Encoder izquierdo: 319
[DEBUG] [1716676554.945342]: Encoder derecho: 318
[DEBUG] [1716676554.950831]: Encoder izquierdo: 320
[DEBUG] [1716676554.956428]: Datos del círculo: posición=(-1.5000303265631278, -0.1303355240579641), radio=0.10415262017986866, distancia=1.5056820147164367
[DEBUG] [1716676554.962405]: Datos del círculo: posición=(-1.4999217307734556, -0.13044844948296822), radio=0.10383171307863055, distancia=1.5055836065854826
[DEBUG] [1716676554.967861]: Datos del círculo: posición=(-1.4998430825313893, -0.13052998518427703), radio=0.10359807207657654, distancia=1.5055123211881951
[DEBUG] [1716676554.975273]: Datos del círculo: posición=(-1.4997863211018019, -0.1305885934626503), radio=0.10342827485797777, distancia=1.5054608562518756
[DEBUG] [1716676554.979501]: Encoder derecho: 319
[DEBUG] [1716676555.002669]: Encoder izquierdo: 321
[DEBUG] [1716676555.010554]: Encoder derecho: 320
[DEBUG] [1716676555.020940]: Datos del círculo: posición=(-1.4997455377257787, -0.130630472074771), radio=0.10330512557999935, distancia=1.5054238599686345
[DEBUG] [1716676555.025380]: Pulsos actuales: izq: 321, der: 320
[DEBUG] [1716676555.029804]: Encoder izquierdo: 322
[DEBUG] [1716676555.037164]: Datos del círculo: posición=(-1.4997164118913067, -0.13066015002533116), radio=0.10321603655965038, distancia=1.5053974195875246
[DEBUG] [1716676555.052083]: Encoder derecho: 321
[DEBUG] [1716676555.054600]: Pulsos actuales: izq: 322, der: 321
[DEBUG] [1716676555.060200]: Datos del círculo: posición=(-1.4996957881570854, -0.13068093353097165), radio=0.10315180709874557, distancia=1.5053786777434532
[DEBUG] [1716676555.068796]: Encoder izquierdo: 323
[DEBUG] [1716676555.082448]: Datos del círculo: posición=(-1.4996813632006993, -0.13069523517612625), radio=0.10310571717294371, distancia=1.5053655488383049
[DEBUG] [1716676555.088196]: Pulsos actuales: izq: 323, der: 321
[DEBUG] [1716676555.095531]: Encoder derecho: 322
[DEBUG] [1716676555.102187]: Encoder izquierdo: 324
[DEBUG] [1716676555.110019]: Encoder derecho: 323
[DEBUG] [1716676555.120224]: Encoder izquierdo: 325
[DEBUG] [1716676555.130057]: Encoder derecho: 324
[DEBUG] [1716676555.185728]: Pulsos actuales: izq: 325, der: 324
[DEBUG] [1716676555.194443]: Datos del círculo: posición=(-1.5025904358460056, -0.12344901242623364), radio=0.10219025117711619, distancia=1.5076530358689633
[DEBUG] [1716676555.201059]: Encoder izquierdo: 326
[DEBUG] [1716676555.206856]: Encoder derecho: 325
[DEBUG] [1716676555.213137]: Encoder izquierdo: 327
[DEBUG] [1716676555.215016]: Pulsos actuales: izq: 327, der: 325
[DEBUG] [1716676555.220893]: Encoder derecho: 326
[DEBUG] [1716676555.236050]: Datos del círculo: posición=(-1.5037031030987114, -0.1206814413399172), radio=0.10183811381788403, distancia=1.5085380447812953
[DEBUG] [1716676555.240330]: Encoder izquierdo: 328
[DEBUG] [1716676555.247899]: Pulsos actuales: izq: 328, der: 326
[DEBUG] [1716676555.248166]: Datos del círculo: posición=(-1.5045646797253296, -0.11854221603153794), radio=0.10157084437971842, distancia=1.5092273296156056
[DEBUG] [1716676555.255422]: Encoder derecho: 327
[DEBUG] [1716676555.265076]: Datos del círculo: posición=(-1.5052138983318535, -0.11693255036412013), radio=0.10137270846290453, distancia=1.5097490192300285
[DEBUG] [1716676555.269362]: Pulsos actuales: izq: 328, der: 327
[DEBUG] [1716676555.270886]: Datos del círculo: posición=(-1.5056959977949205, -0.11573874652318537), radio=0.10122772021153154, distancia=1.5101377073705562
[DEBUG] [1716676555.281314]: Encoder izquierdo: 329
[DEBUG] [1716676555.292127]: Datos del círculo: posición=(-1.5060512152568073, -0.11486023192438621), radio=0.10112244946551234, distancia=1.510424819663074
[DEBUG] [1716676555.296450]: Pulsos actuales: izq: 329, der: 327
[DEBUG] [1716676555.300135]: Datos del círculo: posición=(-1.5063118680213485, -0.11421646941723965), radio=0.10104645044778436, distancia=1.510635907698511
[DEBUG] [1716676555.308102]: Encoder derecho: 328
[DEBUG] [1716676555.328266]: Datos del círculo: posición=(-1.5065027239738047, -0.11374585046779086), radio=0.10099188088404938, distancia=1.5107907121236663
[DEBUG] [1716676555.331946]: Pulsos actuales: izq: 329, der: 328
[DEBUG] [1716676555.336110]: Encoder izquierdo: 330
[DEBUG] [1716676555.353117]: Datos del círculo: posición=(-1.506642328335981, -0.11340230273139598), radio=0.10095295082831597, distancia=1.5109040961617815
[DEBUG] [1716676555.371640]: Encoder derecho: 329
[DEBUG] [1716676555.374274]: Pulsos actuales: izq: 330, der: 329
[DEBUG] [1716676555.394648]: Datos del círculo: posición=(-1.5067444015891662, -0.1131517697692508), radio=0.10092541885303744, distancia=1.510987099455918
[DEBUG] [1716676555.404629]: Datos del círculo: posición=(-1.5068190310910237, -0.1129692286061465), radio=0.10090618863008215, distancia=1.511047861277054
[DEBUG] [1716676555.429447]: Pulsos actuales: izq: 330, der: 329
[DEBUG] [1716676555.439095]: Encoder izquierdo: 331
[DEBUG] [1716676555.455492]: Datos del círculo: posición=(-1.5068736084768242, -0.1128363514712416), radio=0.10089300283271652, distancia=1.5110923579110929
[DEBUG] [1716676555.472710]: Encoder derecho: 330
[DEBUG] [1716676555.477129]: Pulsos actuales: izq: 331, der: 330
[DEBUG] [1716676555.484482]: Datos del círculo: posición=(-1.506913540469812, -0.11273973653077783), radio=0.10088421640526803, distancia=1.5111249672493312
[DEBUG] [1716676555.494408]: Encoder izquierdo: 332
[DEBUG] [1716676555.501382]: Encoder derecho: 331
[DEBUG] [1716676555.510155]: Datos del círculo: posición=(-1.5045788434085368, -0.12215417137727691), radio=0.10440945746793119, distancia=1.509529442448023
[DEBUG] [1716676555.531744]: Encoder izquierdo: 333
[DEBUG] [1716676555.532175]: Pulsos actuales: izq: 333, der: 331
[DEBUG] [1716676555.537159]: Datos del círculo: posición=(-1.5037072224989787, -0.12570680462084136), radio=0.10576245084483225, distancia=1.508952488226046
[DEBUG] [1716676555.554662]: Encoder derecho: 332
[DEBUG] [1716676555.566303]: Datos del círculo: posición=(-1.5030355320630437, -0.12844874966586056), radio=0.10681052284850973, distancia=1.5085141338200183
[DEBUG] [1716676555.571257]: Pulsos actuales: izq: 333, der: 332
[DEBUG] [1716676555.574437]: Datos del círculo: posición=(-1.5025313492175343, -0.13050945176080236), radio=0.10760051178109378, distancia=1.5081887058257561
[DEBUG] [1716676555.584800]: Encoder izquierdo: 334
[DEBUG] [1716676555.601128]: Datos del círculo: posición=(-1.5021582361305748, -0.13203612824976171), radio=0.10818730346682458, distancia=1.5079499015345659
[DEBUG] [1716676555.604631]: Pulsos actuales: izq: 334, der: 332
[DEBUG] [1716676555.612368]: Encoder derecho: 333
[DEBUG] [1716676555.619219]: Encoder izquierdo: 335
[DEBUG] [1716676555.642629]: Datos del círculo: posición=(-1.501884255554515, -0.1331584100470382), radio=0.10861977442204032, distancia=1.5077756727208445
[DEBUG] [1716676555.650488]: Pulsos actuales: izq: 335, der: 333
[DEBUG] [1716676555.665225]: Encoder derecho: 334
[DEBUG] [1716676555.670138]: Datos del círculo: posición=(-1.5016839605994086, -0.13397984620933295), radio=0.10893720627170547, distancia=1.5076489368257462
[DEBUG] [1716676555.684577]: Encoder izquierdo: 336
[DEBUG] [1716676555.689726]: Pulsos actuales: izq: 336, der: 334
[DEBUG] [1716676555.724832]: Datos del círculo: posición=(-1.5015379466928418, -0.1345795247968535), radio=0.10916971647311875, distancia=1.507556915626439
[DEBUG] [1716676555.741165]: Encoder derecho: 335
[DEBUG] [1716676555.743709]: Pulsos actuales: izq: 336, der: 335
[DEBUG] [1716676555.748785]: Datos del círculo: posición=(-1.5014317336105985, -0.13501652736754183), radio=0.10933986187568835, distancia=1.5074902034027675
[DEBUG] [1716676555.769654]: Encoder izquierdo: 337
[DEBUG] [1716676555.782181]: Datos del círculo: posición=(-1.5013546336073642, -0.135334494547449), radio=0.10946433403873203, distancia=1.50744192633372
[DEBUG] [1716676555.786213]: Pulsos actuales: izq: 337, der: 335
[DEBUG] [1716676555.789403]: Encoder derecho: 336
[DEBUG] [1716676555.800636]: Datos del círculo: posición=(-1.5012988021855882, -0.13556547451900638), radio=0.10955540629146626, distancia=1.5074070755192326
[DEBUG] [1716676555.824888]: Encoder izquierdo: 338
[DEBUG] [1716676555.827896]: Pulsos actuales: izq: 338, der: 336
[DEBUG] [1716676555.833279]: Datos del círculo: posición=(-1.501258498114472, -0.13573293176223422), radio=0.10962207291723054, distancia=1.507382004312673
[DEBUG] [1716676555.845002]: Encoder derecho: 337
[DEBUG] [1716676555.872288]: Pulsos actuales: izq: 338, der: 337
[DEBUG] [1716676555.876622]: Datos del círculo: posición=(-1.5012295255410346, -0.135854019569897), radio=0.10967091336931979, distancia=1.5073640578803311
[DEBUG] [1716676555.881577]: Encoder izquierdo: 339
[DEBUG] [1716676555.886734]: Encoder derecho: 338
[DEBUG] [1716676555.893040]: Datos del círculo: posición=(-1.5012088201973106, -0.13594126769245246), radio=0.10970673597003143, distancia=1.5073513028156482
[DEBUG] [1716676555.905425]: Encoder izquierdo: 340
[DEBUG] [1716676555.911497]: Datos del círculo: posición=(-1.5011941449847679, -0.13600382496914645), radio=0.10973305285287974, distancia=1.5073423305084968
[DEBUG] [1716676555.912816]: Pulsos actuales: izq: 340, der: 338
[DEBUG] [1716676555.917695]: Datos del círculo: posición=(-1.5011838669436088, -0.1360483703063666), radio=0.10975242865491887, distancia=1.507336114287316
[DEBUG] [1716676555.925643]: Datos del círculo: posición=(-1.5011767941833762, -0.13607977903611068), radio=0.10976673584383591, distancia=1.5073319056057943
[DEBUG] [1716676555.939767]: Encoder derecho: 339
[DEBUG] [1716676555.963088]: Pulsos actuales: izq: 340, der: 339
[DEBUG] [1716676555.990535]: Datos del círculo: posición=(-1.5011720567732958, -0.13610160979443053), radio=0.10977734141440694, distancia=1.5073291585535
[DEBUG] [1716676556.000346]: Encoder izquierdo: 341
[DEBUG] [1716676556.022171]: Pulsos actuales: izq: 341, der: 339
[DEBUG] [1716676556.023901]: Datos del círculo: posición=(-1.5011690197823377, -0.1361164600645654), radio=0.10978524329743881, distancia=1.5073274749220136
[DEBUG] [1716676556.029894]: Encoder derecho: 340
[DEBUG] [1716676556.037088]: Encoder izquierdo: 342
[DEBUG] [1716676556.046435]: Datos del círculo: posición=(-1.5011672198040016, -0.13612622574362984), radio=0.10979116998486838, distancia=1.5073265642020914
[DEBUG] [1716676556.059367]: Encoder derecho: 341
[DEBUG] [1716676556.062591]: Pulsos actuales: izq: 342, der: 341
[DEBUG] [1716676556.066467]: Datos del círculo: posición=(-1.5019788363127604, -0.12772172311471838), radio=0.10861430269890668, distancia=1.507399503544706
[DEBUG] [1716676556.103254]: Datos del círculo: posición=(-1.5022943857755453, -0.12450798167126437), radio=0.10817150393547482, distancia=1.5074450766222214
[DEBUG] [1716676556.120333]: Encoder izquierdo: 343
[DEBUG] [1716676556.123912]: Pulsos actuales: izq: 343, der: 341
[DEBUG] [1716676556.133639]: Datos del círculo: posición=(-1.5025400463814074, -0.122023511397322), radio=0.1078308845914804, distancia=1.5074867589181584
[DEBUG] [1716676556.142241]: Datos del círculo: posición=(-1.5027259537106625, -0.12015383714092698), radio=0.10757557920844335, distancia=1.5075218859224926
[DEBUG] [1716676556.149113]: Encoder derecho: 342
[DEBUG] [1716676556.165260]: Datos del círculo: posición=(-1.5028645297432959, -0.11876704681746338), radio=0.10738688803027399, distancia=1.5075501338828767
[DEBUG] [1716676556.168532]: Pulsos actuales: izq: 343, der: 342
[DEBUG] [1716676556.171767]: Encoder izquierdo: 344
[DEBUG] [1716676556.187430]: Datos del círculo: posición=(-1.5