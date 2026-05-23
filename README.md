# Navegación de Enjambres Multi-Robot mediante HPPM Jerárquico Descentralizado

Este repositorio contiene la arquitectura central, el entorno de simulación y los scripts de evaluación experimental para un Método de Planificación de Trayectorias Homotópicas (HPPM) jerárquico y descentralizado. Al integrar una jerarquía asimétrica Maestro-Esclavo y una Memoria Topológica Compartida con histéresis paramétrica, este marco resuelve con éxito la singularidad de simetría matemática (Z=0) en colisiones multi-robot.

El sistema ha sido probado rigurosamente tanto en un entorno de gemelo digital interactivo 2D como mediante validación física en laboratorio utilizando plataformas no holonómicas Parallax Scribbler S2 dentro de un espacio de trabajo de 1.8m x 1.8m.

## Arquitectura del Sistema

El proyecto está estructurado en torno a un sistema centralizado de servocontrol visual (visual-servoing) que coordina la planificación y ejecución descentralizada.

1. **Visión y Coordinación (`Real-Time Swarm Navigation.py`):** Actúa como el nodo central. Utiliza una cámara para rastrear marcadores ArUco en los robots, predice sus posiciones usando un Filtro de Kalman y gestiona la Memoria Topológica Compartida. Activa de forma asíncrona el núcleo matemático y envía actualizaciones de trayectoria a los robots mediante sockets TCP.
2. **Núcleo Matemático (`HPPM.c`):** El motor del sistema. Este programa en C calcula las trayectorias homotópicas basándose en las restricciones proporcionadas por el nodo central, devolviendo la trayectoria óptima para cada robot.
3. **Nodo de Comunicación (`Raspberry Code.py`):** Se ejecuta en una Raspberry Pi 3 a bordo de cada robot. Recibe los datos de la trayectoria vía TCP/IP desde el nodo central, calcula las velocidades lineales y angulares necesarias (control cinemático inversa) y envía comandos PWM al robot vía RS232.
4. **Ejecución de Hardware (Archivos `.spin`):** Código cargado directamente en los microcontroladores del Parallax Scribbler S2 para manejar el control de bajo nivel de los motores basado en los comandos seriales recibidos.

## Estructura de Archivos

* **`Real-Time Swarm Navigation.py`:** Script principal en Python para la captura de cámara, detección de ArUco, estimación de estado (Filtro de Kalman), toma de decisiones jerárquica (Memoria Topológica) y transmisión de red.
* **`Raspberry Code.py`:** Script en Python ejecutado en la Raspberry Pi 3. Actúa como servidor TCP, calcula la cinemática inversa y se comunica con el Scribbler S2 por puerto serial.
* **`HPPM.c`:** Código fuente en C para el Método de Planificación de Trayectorias Homotópicas. Compilado como `HPPM.exe` (o ejecutable equivalente en Linux), es llamado por el script principal de Python.
* **`Generate_ArUco.py`:** Script de utilidad para generar los marcadores ArUco utilizados para el seguimiento de los robots.
* **Archivos `.spin` (`RutaDiff.spin`, `s2.spin`, `FullDuplexSerial.spin`):** Código Propeller Spin desplegado en los robots Parallax Scribbler S2 para la ejecución a nivel de hardware.

## Requisitos

### Nodo Central (PC)
* Python 3.x
* OpenCV (`opencv-python`, `opencv-contrib-python`)
* NumPy
* GCC (o compilador C equivalente) para compilar `HPPM.c`

### Nodo Robot (Raspberry Pi 3)
* Python 3.x
* PySerial (`pyserial`)

### Hardware
* Cámara Cenital (calibrada)
* Robots Parallax Scribbler S2
* Raspberry Pi 3 (una por robot)
* Adaptadores Serial a USB

## Configuración y Ejecución

### 1. Núcleo Matemático
Compila el código C en la máquina que ejecutará el nodo central:
```bash
gcc HPPM.c -o HPPM.exe -lm
