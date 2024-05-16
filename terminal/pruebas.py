import RPi.GPIO as GPIO
import time

# Configuración de los pines GPIO según tu configuración
MOTOR_IZQ_ENABLE = 12
MOTOR_IZQ_IN1 = 23
MOTOR_IZQ_IN2 = 24
ENCODER_IZQ = 5

MOTOR_DER_ENABLE = 13
MOTOR_DER_IN3 = 27
MOTOR_DER_IN4 = 22
ENCODER_DER = 6

# Variables de contador
count_izq = 0
count_der = 0
start_time = 0
end_time = 0

# Configuración de GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_IZQ, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_DER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOTOR_IZQ_ENABLE, GPIO.OUT)
GPIO.setup(MOTOR_IZQ_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IZQ_IN2, GPIO.OUT)
GPIO.setup(MOTOR_DER_ENABLE, GPIO.OUT)
GPIO.setup(MOTOR_DER_IN3, GPIO.OUT)
GPIO.setup(MOTOR_DER_IN4, GPIO.OUT)

# Configuración de PWM
pwm_izq = GPIO.PWM(MOTOR_IZQ_ENABLE, 1000)  # Frecuencia de 1 kHz
pwm_der = GPIO.PWM(MOTOR_DER_ENABLE, 1000)  # Frecuencia de 1 kHz
pwm_izq.start(0)  # Iniciar PWM con 0% de duty cycle
pwm_der.start(0)  # Iniciar PWM con 0% de duty cycle

# Función de callback para los encoders
def encoder_callback_izq(channel):
    global count_izq
    count_izq += 1

def encoder_callback_der(channel):
    global count_der
    count_der += 1

# Configuración de interrupciones
GPIO.add_event_detect(ENCODER_IZQ, GPIO.RISING, callback=encoder_callback_izq)
GPIO.add_event_detect(ENCODER_DER, GPIO.RISING, callback=encoder_callback_der)

# Función para mover los motores
def move_motors(pwm_value, duration):
    global start_time, end_time, count_izq, count_der
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_izq.ChangeDutyCycle(pwm_value)
    pwm_der.ChangeDutyCycle(pwm_value)
    
    count_izq = 0
    count_der = 0
    start_time = time.time()
    time.sleep(duration)
    end_time = time.time()
    
    pwm_izq.ChangeDutyCycle(0)  # Detener los motores
    pwm_der.ChangeDutyCycle(0)

# Función principal
def main():
    try:
        while True:
            # Mueve los motores con un duty cycle del 50% durante 2 segundos
            move_motors(50, 2)
            # Calcula el tiempo transcurrido
            duration = end_time - start_time
            # Imprime el conteo de pulsos y el tiempo
            print(f"Conteo de pulsos izquierda: {count_izq}")
            print(f"Conteo de pulsos derecha: {count_der}")
            print(f"Tiempo: {duration} segundos")
            # Calcula y muestra la distancia (ajusta esta parte según tus pruebas)
            distancia = 0.1  # Cambia este valor según la distancia que recorriste en la prueba
            print(f"Distancia recorrida: {distancia} metros")
            # Espera antes de la siguiente prueba
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()


ubuntu@ubuntu:~/catkin_ws/src/my_python_scripts/scripts$ rosrun my_python_scripts calibration.py
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 1: import: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 2: import: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 5: MOTOR_IZQ_ENABLE: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 6: MOTOR_IZQ_IN1: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 7: MOTOR_IZQ_IN2: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 8: ENCODER_IZQ: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 10: MOTOR_DER_ENABLE: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 11: MOTOR_DER_IN3: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 12: MOTOR_DER_IN4: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 13: ENCODER_DER: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 16: count_izq: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 17: count_der: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 18: start_time: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 19: end_time: command not found
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 22: syntax error near unexpected token `GPIO.BCM'
/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/calibration.py: line 22: `GPIO.setmode(GPIO.BCM)'
ubuntu@ubuntu:~/catkin_ws/src/my_python_scripts/scripts$ 

