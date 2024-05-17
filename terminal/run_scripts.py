import subprocess
import keyboard
import os
import time

# Define los comandos
full_launch_commands = [
    "cd ~/catkin_ws",
    "catkin_make",
    "source ~/catkin_ws/devel/setup.bash",
    "roslaunch rplidar_ros rplidar_a2m8.launch",
    "roslaunch hector_slam_launch sinarviz.launch",
    "rosrun my_python_scripts pose_filter.py"
]

relaunch_movement_command = [
    "source ~/catkin_ws/devel/setup.bash",
    "rosrun my_python_scripts ultimo_curva.py"
]

# Función para ejecutar comandos en una nueva terminal
def execute_command_in_terminal(command):
    terminal_command = f"gnome-terminal -- bash -c '{command}; exec bash'"
    subprocess.Popen(terminal_command, shell=True)

# Función para ejecutar los comandos de lanzamiento completo
def execute_full_launch():
    for command in full_launch_commands:
        execute_command_in_terminal(command)
        time.sleep(5)  # Pausa para permitir que cada comando inicie correctamente

# Función para recompilar y lanzar el script de movimiento
def relaunch_movement_script():
    for command in relaunch_movement_command:
        execute_command_in_terminal(command)
        time.sleep(5)  # Pausa para permitir que cada comando inicie correctamente

# Función para resetear Hector SLAM
def reset_hector_slam():
    reset_command = "rosservice call /reset_hector_slam"
    execute_command_in_terminal(reset_command)

# Función para detener el script de movimiento
def stop_movement_script():
    stop_command = "rosnode kill /ultimo_curva.py"
    subprocess.Popen(stop_command, shell=True)

# Loop para detectar teclas
print("Escuchando teclas: 'a' para lanzamiento completo, 's' para relanzar movimiento, 'd' para resetear Hector SLAM, 'f' para detener el movimiento.")
while True:
    if keyboard.is_pressed('a'):
        print("Ejecutando lanzamiento completo...")
        execute_full_launch()
        time.sleep(1)  # Evitar múltiples ejecuciones rápidas

    if keyboard.is_pressed('s'):
        print("Recompilando y relanzando movimiento...")
        relaunch_movement_script()
        time.sleep(1)  # Evitar múltiples ejecuciones rápidas

    if keyboard.is_pressed('d'):
        print("Reseteando Hector SLAM...")
        reset_hector_slam()
        time.sleep(1)  # Evitar múltiples ejecuciones rápidas

    if keyboard.is_pressed('f'):
        print("Deteniendo el script de movimiento...")
        stop_movement_script()
        time.sleep(1)  # Evitar múltiples ejecuciones rápidas

    # Pausa breve para evitar uso intensivo de CPU
    time.sleep(0.1)

#chmod +x run_scripts.py
#./run_scripts.py
