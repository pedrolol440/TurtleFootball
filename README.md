# TurtleFootball
Este repositorio contiene la práctica 3 de la asignatura Robots Móviles y el trabajo realizado durante esta por Pedro Baeza, Carlos Ramos y Leopoldo Cadavid. 

Esta práctica nace durante el principio del mundial de Qatar 2022 en la que se propuso que mediante la cámara Astra de la que dispone el TurtleBot se oriente hacía una persona o "jugador" con una camiseta roja y, al orientarse moverse para tratar de hacerle un "pase" a ese jugador.

Lo primero que se intento, es acceder a la camara y abrir una ventana mediante la librería OpenCV en la que se prentendía visualizar la cámara:

https://github.com/pedrolol440/TurtleFootball/blob/b50c8a7bad8b1708edf03c10674e78a9d1c5f705/Scripts/verCamara.py

En las primeras versiones del código se trato de realizar la detección del color rojo, de la camiseta y el color amarillo, del balón y se moviera hacia el balón al detectar el color amarillo. Aquí se pueden ver algunas de las pruebas que hicimos.

https://user-images.githubusercontent.com/83214961/211801349-519da70b-6e92-46e8-9c1d-8991e57f9634.mp4

https://user-images.githubusercontent.com/83214961/211801359-27cb18e8-4588-49d0-922f-7670e15ad943.mp4

Aquí se puede ver el script usado: 
