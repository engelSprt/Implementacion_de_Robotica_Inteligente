# CHALLENGE 1

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>


**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3002B.502.</p>
<p align="center"> Implementación de robótica inteligente (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 1</p>
<p align="center"> Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos		A01735589</p>
<p align="center">José Ángel Ramírez Ramírez		A01735529</p>
<p align="center">Daniel Ruán Aguilar			A01731921</p>
<p align="center">Fecha de entrega: 18 de abril del 2023</p>


## Resumen

En este reto se refuerzan los conocimientos vistos durante la primera semana que consistía en hacer pruebas con la comunicación del robot que se estará utilizando durante la unidad de formación (puzzlebot). 

Para manipular el robot desde otra computadora se ocupa el protocolo SSH y exportar el ROS master, de esta manera se puede programar los nodos desde nuestra computadora para lograr el objetivo de este primer challenge sin la necesidad de un monitor para la Jetson nano, la cual será la computadora integrada del robot.

En adición la ejecución del robot en físico se hará al mismo tiempo en el simulador de ROS Gazebo, ya que es importante aprender a utilizarlo en caso de que no se tenga el robot para trabajar. Este simulador permite probar y depurar el código antes de implementarlo en un robot físico y de esta manera evitar errores.

## Objetivos

Lo que se busca es crear un nodo para conducir el robot simulado en una ruta cuadrada de una longitud de 2 m por lado. Al mismo tiempo se usa el mismo nodo para mover el robot real en un cuadrado de 2 m de lado utilizando un controlador de lazo abierto y se selecciona la velocidad para terminar el recorrido, finalmente se trazarán segmentos de distancia que el robot rescorrerá tanto en físico y en el simulador.

## Introducción

La inclusión de un robot movil al trabajar con R.O.S. (Robot Operating System) require la manipulación e investigación de diferentes herramientas nuevas para la comunicación entre la jetson nano y otra computadora externa. Las herramientas empleadas en este primer reto son las siguientes.

- Gazebo
- Puzzlebot
- Comunicación SSH
- Ros Master URI

<p align="center">
  <img src="https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/blob/main/challenge1/Challenge1_brainstorm.png" />
</p>

## Solución del problema
Para la solución de este reto se cuentan con un archivo codificados en lenguaje Python, llamado: Square.py. Se comenzará por describir la funcionalidad en el siguiente apartado:

### Squere.py




## Resultados  

**<p align="center"> Video de demostración con la explicación de la ejecución</p>**

En este video se puede ver como el robot realiza un cuadrado como trayectoria y posteriormente se definen trayectorias de diferente inicio y fin.

## Conclusiones

Consideramos que esta primera práctica nos ayudó a terminar de comprender cómo funcionan las herramientas básicas de comunicación del Puzzlebot y ROS, las cuales son importantes e indispensables para desarrollar los futuros retos del curso que implementarán más factores, como la visión por computadora y el control de lazo cerrado. Pudimos resolver los erroes e impedimentos que tuvo el robot de las primeras pruebas en la semana, lo cual nos quitó un poco de tiempo en la resoluión de este reto, sin embargo, pudimos resolver el reto planteado, que en este caso fue la generación de trayectorias con el robot.

