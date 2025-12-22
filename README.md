
![Logo Institucional](https://github.com/JonatanBogadoUNLZ/PPS-Jonatan-Bogado/blob/9952aac097aca83a1aadfc26679fc7ec57369d82/LOGO%20AZUL%20HORIZONTAL%20-%20fondo%20transparente.png)

# Universidad Nacional de Lomas de Zamora – Facultad de Ingeniería

## Desarrollo de un dron multirrotor con inteligencia artificial embarcada para deteccion de personas y focos de incendio con cámara RGB y sensor térmico

---

## Introducción / Objetivo

En la Universidad Nacional de Lomas de Zamora, la Facultad de Ingeniería se dedica a la formación de profesionales en diversas ramas de la ingeniería. Este repositorio corresponde al proyecto desarrollado en el área de **Ingeniería Mecatrónica**, como parte del **Proyecto Final de Carrera**.

El objetivo de este proyecto es el **diseño, construcción e integración de un vehículo aéreo no tripulado (UAV) multirrotor**, capaz de ejecutar misiones autónomas de búsqueda mediante **sensado térmico, visión RGB e inteligencia artificial embarcada**, con transmisión de eventos a una estación de control terrestre.

El proyecto busca demostrar la **viabilidad técnica** de una plataforma de bajo costo orientada a tareas de detección temprana, priorizando estabilidad, confiabilidad y modularidad del sistema.

---

## Índice

- Descripción  
- Instrucciones de Uso  
- Tecnologías Utilizadas  
- Listado de Componentes  
- Esquemáticos  
- Fotos / Videos
- Carpetas del Proyecto  
- Autor  
 

---

## Descripción

Este proyecto se basa en el desarrollo de un **dron multirrotor tipo hexacóptero**, diseñado para realizar misiones autónomas de búsqueda y detección de eventos térmicos y visuales.

El sistema integra:

- Una **plataforma aérea** controlada por un piloto automático (Pixhawk comandado por un ESP32).
- Un **sistema de detección embarcado** basado en Raspberry Pi, encargado del procesamiento de imágenes RGB y térmicas mediante algoritmos de inteligencia artificial.
- Un **microcontrolador ESP32** que actúa como integrador del sistema, coordinando las comunicaciones entre los distintos subsistemas.
- Un enlace de **comunicación LoRa** para la transmisión de eventos relevantes hacia una estación base (*Estación Terrestre*).

La estación terrestre permite definir misiones, visualizar la trayectoria del dron y recibir notificaciones de eventos detectados, incluyendo su localización geográfica.

---

## Instrucciones de Uso

Para utilizar este proyecto, seguir los siguientes pasos generales:

**Paso 1:** Configurar y cargar el firmware correspondiente en el ESP32 y en la controladora de vuelo.

**Paso 2:** Preparar la Raspberry Pi con el sistema operativo y las dependencias necesarias para el procesamiento de imágenes y ejecución de los modelos de IA.

**Paso 3:** Conectar los sensores (cámara RGB y sensor térmico) y verificar su correcto funcionamiento.

**Paso 4:** Ejecutar la aplicación de la Estacion Terrestre desde una computadora en entorno Jupyter, estableciendo la comunicación con el dron mediante el adaptador USB - LoRa.

**Paso 5:** Definir una misión, cargarla en el sistema y realizar las pruebas de vuelo en un entorno controlado.

Asegurarse de contar con los componentes, herramientas y configuraciones detalladas en las secciones siguientes.

---

## Tecnologías Utilizadas

Este proyecto fue desarrollado utilizando las siguientes tecnologías:

### Robótica
- Controladores de vuelo para UAV (Pixhawk)
- Motores brushless y servomotores
- Plataforma multirrotor tipo hexacóptero

### Electrónica
- ESP32
- Sensor térmico (MLX90640)
- Cámara RGB (OV5647)
- Módulos de comunicación LoRa (SX1278)
- Controladores electrónicos de velocidad (ESC)

### Programación
- Python
- C / C++
- Scripts de control y procesamiento

### Plataformas
- MAVLink
- OpenCV
- ONNX Runtime

### Inteligencia Artificial
- Redes neuronales convolucionales
- YOLOv8 para detección de personas y fuego
- Procesamiento de visión por computadora

---

## Listado de Componentes

- Controladora de vuelo Pixhawk
- 6 x ESC de 30 A
- 6 x Motores brushless 2816
- 6 x Hélices 8x4.5 (3 giro derecho y 3 giro izquierdo)
- Batería LiPo 3S 5000 mAh
- Fuente step down XL4015 a 5V
- Microcontrolador ESP32
- Módulo LoRa SX1278
- Servo sg90
- Raspberry Pi  
- Sensor térmico MLX90640  
- Cámara RGB OV5647  
- Chasis impreso en 3D
- 6 x Brazos aluminio portamotores

Para el adaptador USB - LoRa de la base:
- Microcontrolador ESP32
- Módulo LoRa SX1278

---

## Esquemáticos

A continuación se presentan los esquemáticos y diagramas de diseño del sistema:


Esquema eléctrico del dron:

<img width="1500" height="1294" alt="drone" src="https://github.com/user-attachments/assets/3bdfdade-fc21-4515-a349-c243c89ee564" />


Diagrama en bloques del dron:

<img width="621" height="591" alt="Diagrama dron" src="https://github.com/user-attachments/assets/df147e7a-9910-42de-9930-1a5c5da330a5" />



Esquema eléctrico del adaptador USB-LoRa:

<img width="3000" height="1538" alt="puente" src="https://github.com/user-attachments/assets/2921aa3f-d824-4e71-82c9-43b7cd03e9c6" />


Diagrama en bloques del adaptador USB-LoRa:

<img width="601" height="119" alt="Diagrama puente" src="https://github.com/user-attachments/assets/22a31898-7867-4687-9d21-f5d02fd17a3a" />


Diagrama en bloques estación terrestre:

<img width="911" height="611" alt="Diagrama gs" src="https://github.com/user-attachments/assets/c3de5048-7896-4fe7-91da-cc73f6903a75" />



## Fotos / Videos

En esta sección se incluyen imágenes y videos correspondientes al desarrollo y funcionamiento del proyecto:

- Fotografías del dron ensamblado
<img width="1024" height="904" alt="image" src="https://github.com/user-attachments/assets/a8657a35-e360-4dae-ade2-f4d51a0e8704" />

![unnamed](https://github.com/user-attachments/assets/68b30a2c-a352-481c-aaf6-86b9a14eda43)

<img width="660" height="505" alt="image" src="https://github.com/user-attachments/assets/7c16c9e5-efe4-45b6-a1ef-21b08dac71e8" />

![fotodron2](https://github.com/user-attachments/assets/c883afca-a525-49fd-9a1c-a87418a6c9f4)

- Imágenes de pruebas de vuelo

![fotodronvual](https://github.com/user-attachments/assets/abc35068-0f46-405b-ba17-c0758f8c25f5) 

![fotodronvula2](https://github.com/user-attachments/assets/5be659af-1ceb-44fd-9c06-fdbe451f45b8) 

![fotodronvuela3](https://github.com/user-attachments/assets/a3cfb635-f6cd-4c4a-8d10-be3fab45d41a)

### Demostracion del funcionamiento

Seteo de la mision desde la estacion terrestre, verificacion de envios de datos, vuelo, analisis con resultado negativo y positivo (con imagenes capatadas por camara RGB e infrarroja)

https://drive.google.com/file/d/1ojA_1EPUBh213Cx6VwPE2h_yf2Y8xk58/view?usp=drive_link

https://youtu.be/2DbrhcWoiKQ


---

## Carpetas del Proyecto

La estructura del repositorio es la siguiente:

- **Adaptador USB-LoRa:** Codigo del adapatador para ESP32.
- **Camara termografica:** Codigo utilizado en el M5stick y Raspberry Pi.
- **Codigos camara:** Codigo utilizado para la camara RGB en Rapsberry Pi.
- **Entrenamiento IA:** Imagenes utilizada para entrenar la inteligencia artificial.
- **Esquemas electricos:** Esquemas de los componentes utilizados.
- **Estacion terrestre:** Versiones de la aplicacion de la estacion terrestre, codigo fuente a ejecutar en Jupyter.
- **Funcionando para final - comportameinto full:** Programas definitivos con el comportamiento correcto.
- **Funcionando para regularidad:** Programas con vuelo lineal a un punto.
- **Hojas de datos:** Hojas de datos de los componentes utilizados.
- **Informes:** Documentación técnica, informes, presentacion, cronogramas y material académico relacionado - leer informe e informe complementario.
- **Modelado 3D:** Archivos editables en SolidWorks con el ensamble y piezas del dron.  
- **Pixhawk:** Código fuente del sistema para ESP32.
- **Videos demostracion funcionamiento:** Imágenes y videos del desarrollo y pruebas, videos de misiones.  


---

## Autores

Este proyecto fue realizado por 

**Fernández Malenotti, Ignacio** 
fmalenotti.ignacio@gmail.com

**Moscariello, Christian**
chris_mosca7@hotmail.com

**Yacono, Emiliano**
emilianoyacono@gmail.com

