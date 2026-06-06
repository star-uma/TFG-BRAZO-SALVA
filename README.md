## Repositorio del manipulador STAR-UMA

Este repositorio contiene la implementación de un sistema de teleoperación para un manipulador robótico desarrollado como Trabajo de Fin de Grado (TFG).

El sistema permite controlar el brazo en el espacio articular y cartesiano mediante velocidades, utilizando un joystick para realizar movimientos intuitivos y tareas de manipulación como el apilado de bloques.


## Requisitos
- ROS2 (probado en Humble)
- Linux (Ubuntu 22.04)

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

## Autor
Salvador Patricio Lázaro Herrero.
Correo: salvapatricio@uma.es

Trabajo de Fin de Grado – Ingeniería Electrónica, Robótica y Mecatrónica.

## Licencia
Proyecto desarrollado con fines académicos (Opensource) 
