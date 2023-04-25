# CHALLENGE 2

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>


**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3002B.502.</p>
<p align="center"> Implementación de robótica inteligente (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 2</p>
<p align="center"> Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos		A01735589</p>
<p align="center">José Ángel Ramírez Ramírez		A01735529</p>
<p align="center">Daniel Ruán Aguilar			A01731921</p>
<p align="center">Fecha de entrega: 25 de abril del 2023</p>


## Resumen

En este reto se aplican los conocimientos vistos durante la semana, los cuales le dan continuidad al primer reto, pero esta vez mostrando el comportamiento de un control diseñado de lazo cerrado en robótica.

Para lograr la programación de este nuevo comportamiento en el robot se reutilizaron los conceptos requeridos en la implementación del control vistos previamente, tales como el cálculo del error generado, obtener la posición del robot, tipos de controladores, etc. 

## Objetivos

El objetivo consiste en utilizar un control PID para mover el robot a diferentes posiciones en el espacio planteado. Nuevamente se debe conducir el robot en una ruta cuadrada de una longitud de 2 m por lado, posteriormente se se debe crea un nodo generador de rutas, que publique la ruta actual y
próximo objetivo una vez que el robot complete el objetivo actual.

Estas trayectorias se implementan utilizando el Gazebo Puzzlebot Simulator y físicamente en el robot en tiempo real. Para esto se crearán los nodos y paquetes en ROS necesarios que se ecplicarán en la solución del problema.

## Introducción

A partir del reto 1 que involucraba la movilidad del puzzlebot en lazo abierto, se investigaron y aplicaron los siguientes conceptos para la resolución de este nuevo reto.
- Control en lazo cerrado para un robot móvil
- PID aplicado a un robot móvil diferencial
- Cálculo del error
- Robustez de un controlador

<p align="center">
  <img src=" " />
</p>

## Solución del problema
Para la solución de este reto se cuentan con tres archivos codificados en lenguaje Python, llamados: path_generator.py, controller.py y .py Se comenzará por describir la funcionalidad de cada uno en el siguiente apartado:

### path_generator.py

### controller.py


## Resultados  

**<p align="center"> Videos de demostración en el robot</p>**

**Trayectoria Cuadrada**




**<p align="center"> Videos de demostración del robot en el simulador Gazebo</p>**

**Trayectoria Cuadrada**



En estos videos se puede ver como el robot realiza la trayectoria tanto en físico y en el simulador.

## Conclusiones

Para la solución del reto se necesitó saber en todo momento la posición del robot, se implemento un controlador PID ya que había funcionando previamente en los retos del bloque anterior. Con el generador de trayectorias nos dimos cuenta que se puede automatizar los procesos en el robot y que será de gran ayuda en el reto final.
