## Repositorio del manipulador STAR-UMA

Este repositorio contiene la implementación de un sistema de teleoperación para un manipulador robótico desarrollado como Trabajo de Fin de Grado (TFG).

El sistema permite controlar el brazo en el espacio articular y cartesiano mediante velocidades, utilizando un joystick para realizar movimientos intuitivos y tareas de manipulación como el apilado de bloques.


## Requisitos
- Linux (Ubuntu 22.04)
- ROS2 (probado en Humble) (se puede instalar con las instrucciones de la pag oficial) 
- Python 3.1 instalado


Dispositivos necesarios:

- Joystick → /dev/input/js0
- Interfaz Dynamixel → /dev/ttyUSB0 o /dev/ttyUSB1
- Interfaz CAN → /dev/ttyACM0

## Instalación

- Crear un workspace de ROS2 (si no existe):
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src

- Clonar el repositorio:
  git clone <URL_DEL_REPOSITORIO>

- Compilar:
  cd ~/ros2_ws
  colcon build
  source install/setup.bash

- Instalar paquetes:
  sudo apt update
  sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-hardware-interface \
  ros-humble-realtime-tools \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-effort-controllers \
  ros-humble-velocity-controllers \
  ros-humble-forward-command-controller \
  ros-humble-std-msgs \
  ros-humble-std-srvs \
  ros-humble-sensor-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-geometry-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-msgs \
  ros-humble-tf2-geometry-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-urdf-parser-plugin \
  ros-humble-kdl-parser \
  ros-humble-joy \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-launch \
  ros-humble-launch-ros \
  ros-humble-launch-xml \
  ros-humble-launch-yaml \
  ros-humble-pluginlib \
  ros-humble-dynamixel-sdk
  sudo apt install ros-humble-dynamixel-ros2
  sudo apt install ros-humble-dynamixel-sdk
  sudo apt install ros-humble-launch-param-builder

- También hay que pegar los archivos (contenidos en la carpeta "Añadidos") en la carpeta correspondiente:
  1. Pegar "libservo_api.so"  en la dirección: Equipo / usr / local / lib
  2. Pegar la carpeta "servo_api" en la dirección: Equipo / usr / local / include

  Seguramente tendrás que hacerlo con la terminal porque te pedirá derechos de admin.

## Uso

Lanzar el sistema con:

ros2 launch bringup bringup.launch.py

## ⚠️ Notas importantes

Antes de ejecutar, asegúrate de que los dispositivos están correctamente detectados:

Joystick → /dev/input/js0
Dynamixel → /dev/ttyUSB0 o /dev/ttyUSB1
CAN → /dev/ttyACM0

Puedes comprobarlo con:

ls /dev/

## ❗Resolución de problemas


- En algunos casos, puede ser necesario relanzar el sistema hasta que todos los dispositivos se inicialicen correctamente:

- Si ves que el brazo va muy lento ( Los dinamixel ralentizan la actualización de pose) tienes que acelerar la velocidad del puerto USB de tu ordenador:
  1. cd /dev
  2. sudo nano /sys/bus/usb-serial/devices/ttyUSB1/latency_timer ( escribes dentro un 0 )( ttyUSB1 o ttyUSB0 depende el que te detecte)

## Demostración

El sistema permite la teleoperación del robot y la ejecución de tareas como el apilado de bloques en tiempo real.
https://www.youtube.com/watch?v=vmkai2tdlqk

## Autor
Salvador Patricio Lázaro Herrero.
Correo: salvapatricio@uma.es

Trabajo de Fin de Grado – Ingeniería Electrónica, Robótica y Mecatrónica.

## Licencia
Proyecto desarrollado con fines académicos (Opensource) 
