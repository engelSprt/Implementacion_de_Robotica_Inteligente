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

En este reto se refuerzan los conocimientos vistos durante la primera semana que consistía en hacer pruebas con la comunicación del robot que se estará utilizando durante la unidad de formación (puzzlebot). 

Para manipular el robot desde otra computadora se ocupa el protocolo SSH y exportar el ROS master, de esta manera se puede programar los nodos desde nuestra computadora para lograr el objetivo de este primer challenge sin la necesidad de un monitor para la Jetson nano, la cual será la computadora integrada del robot.

En adición la ejecución del robot en físico se hará al mismo tiempo en el simulador de ROS Gazebo, ya que es importante aprender a utilizarlo en caso de que no se tenga el robot para trabajar. Este simulador permite probar y depurar el código antes de implementarlo en un robot físico y de esta manera evitar errores.

## Objetivos

Lo que se busca es crear un nodo para conducir el robot simulado en una ruta cuadrada de una longitud de 2 m por lado. Al mismo tiempo se usa el mismo nodo para mover el robot real en un cuadrado de 2 m de lado utilizando un controlador de lazo abierto y se selecciona la velocidad para terminar el recorrido, finalmente se trazarán segmentos de distancia que el robot rescorrerá tanto en físico y en el simulador.

## Introducción

Como continuación del reto 1 para la movilidad del puzzlebot se investigaron y aplicaron los siguientes conceptos para su resolución.
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

Consideramos que esta primera práctica nos ayudó a terminar de comprender cómo funcionan las herramientas básicas de comunicación del Puzzlebot y ROS, las cuales son importantes e indispensables para desarrollar los futuros retos del curso que implementarán más factores, como la visión por computadora y el control de lazo cerrado. Pudimos resolver los erroes e impedimentos que tuvo el robot de las primeras pruebas en la semana, lo cual nos quitó un poco de tiempo en la resoluión de este reto, sin embargo, pudimos resolver el reto planteado, que en este caso fue la generación de trayectorias con el robot.

