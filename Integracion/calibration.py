#!/usr/bin/env python3


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
def move_motors(pwm_valued,pwm_valuei, duration):
    global start_time, end_time, count_izq, count_der
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_izq.ChangeDutyCycle(pwm_valuei)
    pwm_der.ChangeDutyCycle(pwm_valued)
    
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
            move_motors(59,58, 2)
            # Calcula el tiempo transcurrido
            duration = end_time - start_time
            # Imprime el conteo de pulsos y el tiempo
            print(f"Conteo de pulsos izquierda: {count_izq}")
            print(f"Conteo de pulsos derecha: {count_der}")
            print(f"Tiempo: {duration} segundos")
            # Calcula y muestra la distancia (ajusta esta parte según tus pruebas)
            distancia = 0.5  # Cambia este valor según la distancia que recorriste en la prueba
            print(f"Distancia recorrida: {distancia} metros")
            # Espera antes de la siguiente prueba
            time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
