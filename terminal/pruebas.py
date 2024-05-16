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

# Constantes k basadas en tus datos (pulsos por centímetro)
k_izq = 29.23
k_der = 29.57

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
def move_motors(distancia_cm):
    global start_time, end_time, count_izq, count_der
    GPIO.output(MOTOR_IZQ_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IZQ_IN2, GPIO.LOW)
    GPIO.output(MOTOR_DER_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_DER_IN4, GPIO.LOW)
    pwm_izq.ChangeDutyCycle(88)
    pwm_der.ChangeDutyCycle(90)
    
    count_izq = 0
    count_der = 0
    start_time = time.time()
    
    pulsos_deseados_izq = distancia_cm * k_izq
    pulsos_deseados_der = distancia_cm * k_der
    
    print(f"Inicio del movimiento: Pulsos deseados izq: {pulsos_deseados_izq}, der: {pulsos_deseados_der}")
    
    while count_izq < pulsos_deseados_izq or count_der < pulsos_deseados_der:
        print(f"Pulsos actuales: izq: {count_izq}, der: {count_der}")
        time.sleep(0.01)  # Ajusta este valor según sea necesario
    
    end_time = time.time()
    
    pwm_izq.ChangeDutyCycle(0)  # Detener los motores
    pwm_der.ChangeDutyCycle(0)
    print("Motores detenidos")

# Función principal
def main():
    try:
        # Mueve los motores para una distancia deseada de 100 cm (1 metro)
        distancia_deseada_cm = 200  # Reemplazar con la distancia deseada en cm
        move_motors(distancia_deseada_cm)
        # Calcula el tiempo transcurrido
        duration = end_time - start_time
        # Imprime el conteo de pulsos y el tiempo
        print(f"Conteo de pulsos izquierda: {count_izq}")
        print(f"Conteo de pulsos derecha: {count_der}")
        print(f"Tiempo: {duration} segundos")
        print(f"Distancia deseada: {distancia_deseada_cm} cm")
        print(f"Distancia calculada izquierda: {count_izq / k_izq} cm")
        print(f"Distancia calculada derecha: {count_der / k_der} cm")
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
