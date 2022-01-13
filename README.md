# DRONE PROJECT RIS

# Objetivo

El objetivo es conducir el dron y aterrizar frente al aruco marker (Figura 1) utilizando dos modalidades de navegación:
a) Navegación global a través de waypoints
b) Navegación servo basada en la posición

![image](https://user-images.githubusercontent.com/61427246/149408706-aceb4df8-2871-4e8b-839d-2916d205c863.png)
Figura 1. Configuración inicial del drone y el marker aruco.

Para ello se ha construido el repositorio trajectory_control_node. 

# Configuración y ejecución del nodo trayectory_control_node
1. Instalacion y configuración según: https://asantamarianavarro.gitlab.io/code/teaching/ris/ris_project_instructions/#installation

2. Descargar el repositorio trajectory_control_node y guardarlo en la dirección <YOUR_PATH>/ris_project_ws/src

3. Construir el repositorio
   $ cd <YOUR_PATH>/ris_project_ws/src
   $ catkin build trayectory_control_node
   $ source devel/setup.bash

4. Lanzar  launch de simulación 
   $ roslaunch ris_drone_mission ris_drone_mission.launch

5. Lanzar nodo trayectory_control_node (en un nuevo terminal)
   $ rosrun trayectory_control_node trayectory_control_node
   
# Simulación
https://youtu.be/FvbQ9OkvrU0
