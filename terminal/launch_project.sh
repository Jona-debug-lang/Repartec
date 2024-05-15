#!/bin/bash

# Navegar al directorio del workspace de catkin
cd ~/catkin_ws

# Construir el workspace de catkin
catkin_make

# Fuente del ambiente de ROS
source ~/catkin_ws/devel/setup.bash

# Lanzar el nodo del LIDAR
roslaunch rplidar_ros rplidar_a2m8.launch

# Lanzar el nodo de Hector SLAM
roslaunch hector_slam_launch sinarviz.launch

# Ejecutar el script Python
rosrun my_python_scripts pose_filter.py


