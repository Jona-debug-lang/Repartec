#!/usr/bin/env python3
from pynput import keyboard
import subprocess
import threading
import time
import signal

# Ruta al setup.bash
setup_command = "source ~/catkin_ws/devel/setup.bash"

# Diccionario para almacenar los procesos de los comandos y programas
procesos_activos = {}

# Diccionario de comandos personalizados
comandos_personalizados = {
    '!': 'roscore',
    '@': 'cd ~/catkin_ws && sleep 5 && catkin_make',
    '$': 'roslaunch rplidar_ros rplidar_a2m8.launch',
    '%': 'roslaunch hector_slam_launch sinarviz.launch',
    '^': 'roslaunch obstacle_detector prueba1.launch'
}

# Diccionario de scripts personalizados
scripts_personalizados = {
    '&': '/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/evasion_estatico.py',
    '*': '/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/pose_filter.py',
    '9': '/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/full4.py',
    ')': '/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/interfaz.py'
}

# Flag para controlar el estado del contador
contador_activo = False

def iniciar_contador():
    global contador_activo
    contador_activo = True
    for i in range(5, 0, -1):
        print(f"Esperando {i} segundos")
        time.sleep(1)
    contador_activo = False

def ejecutar_proceso(comando, clave):
    global contador_activo
    if clave in procesos_activos:
        # Detener el proceso si ya está en ejecución
        pid = procesos_activos[clave].pid
        subprocess.run(['kill', '-9', str(pid)])
        del procesos_activos[clave]
        print(f"Proceso {clave} detenido")
    else:
        if not contador_activo:
            # Iniciar el proceso si no está en ejecución y no hay contador activo
            full_command = f"{setup_command} && {comando}; exec bash"
            proceso = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', full_command])
            procesos_activos[clave] = proceso
            print(f"Proceso {clave} ejecutado")
            threading.Thread(target=iniciar_contador).start()

def detener_todos_los_procesos():
    global contador_activo
    if not contador_activo:
        for clave, proceso in procesos_activos.items():
            subprocess.run(['kill', '-9', str(proceso.pid)])
            print(f"Proceso {clave} detenido")
        procesos_activos.clear()
        print("Todos los procesos han sido detenidos forzosamente")
        # Cerrar todas las ventanas de gnome-terminal
        subprocess.run(['pkill', '-f', 'gnome-terminal'])
    else:
        print("Espera a que el contador termine antes de detener todos los procesos")

def on_press(key):
    global contador_activo
    try:
        if key.char == 'y':
            detener_todos_los_procesos()
        elif key.char in scripts_personalizados:
            if not contador_activo:
                ejecutar_proceso(f'python3 {scripts_personalizados[key.char]}', key.char)
        elif key.char in comandos_personalizados:
            if not contador_activo:
                ejecutar_proceso(comandos_personalizados[key.char], key.char)
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        detener_todos_los_procesos()
        return False

if __name__ == "__main__":
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
 