#!/usr/bin/env python3
from pynput import keyboard
import rospy
from std_msgs.msg import String
import subprocess
import threading
import time

# Ruta al setup.bash
setup_command = "source ~/catkin_ws/devel/setup.bash"

# Diccionario para almacenar los procesos de los comandos y programas
procesos_activos = {}
procesos_reinicio = {}
proceso_secuencial_activo = None
secuencia_completa = False  # Flag para rastrear el estado de finalización de la secuencia

# Lista de comandos personalizados para ejecutar en secuencia
comandos_secuenciales = [
    'cd ~/catkin_ws && sleep 1 && catkin_make',
    'roslaunch rplidar_ros rplidar_a2m8.launch',
    'roslaunch hector_slam_launch sinarviz.launch',
    'rosrun my_python_scripts pose_filter.py',
    'roslaunch obstacle_detector prueba1.launch'
]

# Comandos a reiniciar
comandos_a_reiniciar = [
    'roslaunch hector_slam_launch sinarviz.launch',
    'roslaunch obstacle_detector prueba1.launch'
]

# Diccionario de scripts personalizados
scripts_personalizados = {
    'a': '/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/Repartec/FINAL/experimento.py',
    'b': '/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/Repartec/FINAL/interfaz.py'
}

# Diccionario de rostopics personalizados
rostopics_personalizados = {
    't': '/topic1',
    'y': '/topic2',
    'u': '/topic3'
}

# Flag para controlar el estado del contador
contador_activo = False

def iniciar_contador():
    global contador_activo
    contador_activo = True
    for i in range(10, 0, -1):
        print(f"Esperando {i} segundos")
        time.sleep(1)
    contador_activo = False

def ejecutar_proceso(comando, nombre_ventana, mostrar_terminal=False):
    global contador_activo
    if nombre_ventana in procesos_activos:
        # Detener el proceso si ya está en ejecución
        proceso = procesos_activos[nombre_ventana]
        proceso.terminate()
        proceso.wait()
        del procesos_activos[nombre_ventana]
        print(f"Proceso en ventana {nombre_ventana} detenido")
        rospy.loginfo(f"Proceso {nombre_ventana} detenido")
    else:
        if not contador_activo:
            # Iniciar el proceso si no está en ejecución y no hay contador activo
            full_command = f"{setup_command} && {comando}"
            if mostrar_terminal:
                proceso = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', full_command])
            else:
                proceso = subprocess.Popen(['bash', '-c', full_command], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            procesos_activos[nombre_ventana] = proceso
            print(f"Proceso ejecutado en ventana {nombre_ventana}: {comando}")
            rospy.loginfo(f"Proceso {nombre_ventana} ejecutado")
            threading.Thread(target=iniciar_contador).start()

def ejecutar_rostopic_echo(nombre_ventana, topic):
    comando = f"rostopic echo {topic} -n 1 > /tmp/rostopic_output"
    ejecutar_proceso(comando, nombre_ventana, mostrar_terminal=True)
    time.sleep(2)  # Espera para que el comando tenga tiempo de ejecutarse
    with open('/tmp/rostopic_output', 'r') as file:
        output = file.read().strip()
        if output:
            print(f"Escuchando rostopic echo {topic}: {output}")
        else:
            print(f"No se está escuchando rostopic echo {topic}")

def ejecutar_comandos_secuenciales():
    global proceso_secuencial_activo, secuencia_completa
    secuencia_completa = False
    for i, comando in enumerate(comandos_secuenciales):
        if proceso_secuencial_activo:
            print(f"Ejecutando {comando}...")
            nombre_ventana = f'comando_{i+1}'
            full_command = f"{setup_command} && {comando}"
            proceso = subprocess.Popen(['bash', '-c', full_command], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            procesos_activos[nombre_ventana] = proceso
            print(f"Proceso {comando} ejecutado")
            time.sleep(10)  # Espera 10 segundos antes de continuar con el siguiente comando
            print(f"Proceso {comando} terminado")
            if not proceso_secuencial_activo:
                break
    proceso_secuencial_activo = None
    secuencia_completa = True
    print("Todos los comandos secuenciales han sido ejecutados")

def detener_todos_los_procesos():
    global contador_activo, proceso_secuencial_activo, secuencia_completa
    if not contador_activo:
        proceso_secuencial_activo = None
        secuencia_completa = False
        for nombre_ventana, proceso in procesos_activos.items():
            proceso.terminate()
            proceso.wait()
            print(f"Proceso {nombre_ventana} detenido")
        procesos_activos.clear()
        print("Todos los procesos han sido detenidos forzosamente")
        # Cerrar todas las ventanas de gnome-terminal
        subprocess.run(['pkill', '-f', 'gnome-terminal'])
    else:
        print("Espera a que el contador termine antes de detener todos los procesos")

def manejar_reinicio(msg):
    if msg.data == "reinicio":
        rospy.loginfo("Reinicio recibido. Terminando y reiniciando procesos.")
        detener_procesos_reinicio()
        iniciar_procesos_reinicio()

def detener_procesos_reinicio():
    for nombre_ventana, proceso in procesos_reinicio.items():
        proceso.terminate()
        proceso.wait()
        print(f"Proceso {nombre_ventana} detenido")
    procesos_reinicio.clear()
    rospy.loginfo("Todos los procesos de reinicio han sido detenidos")

def iniciar_procesos_reinicio():
    for i, comando in enumerate(comandos_a_reiniciar):
        nombre_ventana = f'reinicio_{i+1}'
        threading.Thread(target=ejecutar_comando_reinicio, args=(comando, nombre_ventana)).start()
        time.sleep(10)  # Espera 10 segundos antes de continuar con el siguiente comando
        print(f"Proceso de reinicio {comando} terminado")
    rospy.loginfo("Todos los procesos de reinicio han sido iniciados")
    rospy.loginfo("Reinicio completado. Publicando 'GRACIAS'.")
    pub.publish("GRACIAS")

def ejecutar_comando_reinicio(comando, nombre_ventana):
    full_command = f"{setup_command} && {comando}"
    proceso = subprocess.Popen(['bash', '-c', full_command], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    procesos_reinicio[nombre_ventana] = proceso
    print(f"Proceso {comando} ejecutado")
    stdout, stderr = proceso.communicate()
    if proceso.returncode == 0:
        print(f"Proceso {comando} terminado con éxito")
    else:
        print(f"Proceso {comando} terminado con errores: {stderr.decode()}")

def on_press(key):
    global contador_activo, proceso_secuencial_activo, secuencia_completa
    if not contador_activo:
        try:
            if key.char == '1':
                detener_todos_los_procesos()
            elif key.char == '2':
                if proceso_secuencial_activo:
                    print("La secuencia ya está en ejecución.")
                elif secuencia_completa:
                    print("Reiniciando la secuencia de comandos secuenciales...")
                    detener_todos_los_procesos()
                    proceso_secuencial_activo = True
                    threading.Thread(target=ejecutar_comandos_secuenciales).start()
                else:
                    proceso_secuencial_activo = True
                    threading.Thread(target=ejecutar_comandos_secuenciales).start()
            elif key.char in scripts_personalizados:
                ejecutar_proceso(f'python3 {scripts_personalizados[key.char]}', f'script_{key.char}')
            elif key.char in rostopics_personalizados:
                ejecutar_rostopic_echo(f'rostopic_{key.char}', rostopics_personalizados[key.char])
        except AttributeError:
            pass
    else:
        if key.char in scripts_personalizados:
            # Reiniciar el script
            nombre_ventana = f'script_{key.char}'
            ejecutar_proceso(f'python3 {scripts_personalizados[key.char]}', nombre_ventana)
            print(f"Reiniciando script {nombre_ventana}")

def on_release(key):
    if key == keyboard.Key.esc:
        detener_todos_los_procesos()
        return False

if __name__ == "__main__":
    rospy.init_node('control_comandos')
    pub = rospy.Publisher('/boton', String, queue_size=10)
    rospy.Subscriber('/comando_reinicio', String, manejar_reinicio)

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    rospy.spin()
