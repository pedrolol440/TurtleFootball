# TurtleFootball
Este repositorio contiene la práctica 3 de la asignatura Robots Móviles y el trabajo realizado durante esta por Pedro Baeza, Carlos Ramos y Leopoldo Cadavid. 

Esta práctica nace durante el principio del mundial de Qatar 2022 en la que se propuso que mediante la cámara Astra de la que dispone el TurtleBot se oriente hacía una persona o "jugador" con una camiseta roja y, al orientarse moverse para tratar de hacerle un "pase" a ese jugador.

Lo primero que se intento, es acceder a la camara y abrir una ventana mediante la librería OpenCV en la que se prentendía visualizar la cámara:

[VerCamara.py code](https://github.com/pedrolol440/TurtleFootball/blob/b2d81bce231dedcae2abcad0811543c91382c81d/Scripts/verCamara.py)

En las primeras versiones del código se trato de realizar la detección del color rojo, de la camiseta y el color amarillo, del balón y se moviera hacia el balón al detectar el color amarillo. Aquí se pueden ver algunas de las pruebas que hicimos.

https://user-images.githubusercontent.com/83214961/211801349-519da70b-6e92-46e8-9c1d-8991e57f9634.mp4

https://user-images.githubusercontent.com/83214961/211801359-27cb18e8-4588-49d0-922f-7670e15ad943.mp4

Aquí se puede ver el script usado: 

[redTracking.py](https://github.com/pedrolol440/TurtleFootball/blob/8d85a298ce28f669f9b3d25606ab63ee212cacbe/Scripts/redTracking.py)

A partir de este momento se necesitaba que el robot se posicionara de una manera en la que al empujar el balón le llegará al jugador que lleve una camiseta roja, para ello se comenzaron a probar varias ideas para la orientación del TurtleBot. De muchas ideas se empezó a trabajar a partir de esta: 

[giro_prueba_odom.py](https://github.com/pedrolol440/TurtleFootball/blob/a90a78e25fbfcaf5cb84aaa6f085ff1dbef41576/Scripts/prueba_giro_odom.py)
