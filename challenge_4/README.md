# CHALLENGE 4

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>

**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3002B.502.</p>
<p align="center">Implementación de robótica inteligente (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 4</p>
<p align="center">Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos     A01735589</p>
<p align="center">José Ángel Ramírez Ramírez    A01735529</p>
<p align="center">Daniel Ruán Aguilar           A01731921</p>
<p align="center">Fecha de entrega: 28 de Mayo del 2023</p>


## Resumen

 


## Objetivos

Continuando con la serie de mini-retos semanales, en esta ocasión se nos ha planteado usar nuevamente los conceptos de vison computacional de manera que con base en el mini-reto anterior, seamos capaces de agregar una nueva capa que nos permita hacer seguimiento de lineas usando la camara incluida en el puzzlebot.
Para este mini-reto se espera que el puzzebot sea capaz y tenga la robustez necesaria para seguir una linea sobre una pista, misma que será diseñada por los integrantes del equipo y que debe cumplir con las medidas establecidad por el socioformador en la presentacion de este mini-reto, ademas de esto, el algoritmo debe mantener la deteccion de un semaforo con los distintos colores y la respectiva accion para cada color. 


## Introducción 

Los automóviles autónomos hoy en día han cobrado mucha relevancia y aún hoy en día para muchas personas pareciera que estos funcionan con magia; sin embargo, detrás de estos automóviles existe un tema bastante complejo pero fundamental para que puedan ser autónomos. Es la visión computacional, la cual es un campo de estudio que se enfoca en enseñar a las computadoras a "ver" y comprender el contenido visual de imágenes o videos, permitiéndoles realizar tareas como reconocimiento de objetos, detección de patrones, seguimiento de movimientos y análisis de escenas. En el caso de los vehículos autónomos se tienen que tomar en cuenta distintos puntos, de manera que el sistema trabaje de la manera más eficiente. Algunos de los puntos más importantes para este reto son los siguientes:

* Detección de líneas y contornos en una imagen.
* Algoritmos implementados para el seguimiento de líneas y trayectorias.
* Robustez en sistemas de procesamiento de imágenes, etc.

![mapa mental reto4](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/100887194/f4a8d9a2-09ab-49c0-9422-c32dca103cb1)



## Solución


## Resultados




## Conclusiones

Un problema que se presentó fue al momento de pasar nuestros códigos probados en nuestros ordenadores a la jetson, ya que la cámara del puzzlebot es de una menor calidad que las de las computadoras, por lo que no segmentaba de igual manera las líneas negras, entonces se tuvo que modificar e ir cambiando las variables dentro de la jetson para acoplarse a las nuevas condiciones de la cámara-raspberry pi.
