# Guía – Creación de nodos ROS 2 con Python

## Objetivo

El objetivo de esta guía es aprender a crear nodos ROS 2 en Python desde cero, partiendo de los conceptos básicos (publisher y subscriber) hasta construir un nodo completo de teleoperación por teclado para el PhantomX Pincher.

La guía está organizada como una serie de videos. Cada archivo en esta carpeta corresponde a un video y cubre un concepto específico de forma aislada, de modo que al llegar al nodo final todo ya haya sido explicado por partes.

---

## Archivos de la serie

### Video 1 – Publisher: `ejemplo_publisher.py`

Introduce los conceptos fundamentales de un nodo ROS 2:

- Qué es un nodo y cómo se define con `rclpy.Node`
- Qué es un tópico y cómo publicar en él con `create_publisher`
- Cómo ejecutar código de forma periódica con `create_timer`
- Cómo construir y enviar un mensaje (`std_msgs/String`)

El nodo publica el nombre de un estado del robot en `/named_target` cada 3 segundos.

### Video 2 – Subscriber: `ejemplo_subscriber.py`

Muestra cómo escuchar un tópico:

- Cómo suscribirse con `create_subscription`
- Qué es un callback y cuándo se ejecuta
- Estructura del mensaje `sensor_msgs/JointState` (campos `name`, `position`, `velocity`)

El nodo escucha `/joint_states` e imprime la posición de cada joint en radianes.

### Video 3 – Lectura de teclado: `ejemplo_teclado.py`

Script Python puro, sin ROS 2. Explica cómo leer teclas sin necesitar presionar Enter:

- Qué es el modo raw del terminal y en qué se diferencia del modo normal
- Cómo activarlo con `tty.setraw()` y `termios`
- Por qué es obligatorio restaurar la terminal al salir (`finally`)
- Cómo leer un carácter a la vez con `sys.stdin.read(1)`

> Este script se puede correr en cualquier máquina con Python, sin necesidad de ROS 2.

### Video 4 – Detección de movimiento: `ejemplo_deteccion_movimiento.py`

Extiende el subscriber del Video 2 añadiendo la lógica para saber si el robot se está moviendo o se detuvo:

- Por qué se comparan posiciones consecutivas (delta)
- Cómo elegir el umbral de movimiento (`UMBRAL_MOVIMIENTO`)
- Por qué se necesita una bandera `_robot_moviendose` (el robot tarda en arrancar)
- Por qué no basta con una sola lectura quieta (`LECTURAS_ESTABLES`)

El nodo imprime en consola cuándo el robot arranca y cuándo se detiene.

### Video 5 – Teleop completo: `teleop_keyboard.py`

> El archivo final se encuentra en:
> `phantom_ws/src/pincher_control/pincher_control/teleop_keyboard.py`

Combina todos los conceptos anteriores en un único nodo:

- Publisher en `/named_target` (Video 1)
- Subscriber en `/joint_states` con detección de movimiento (Videos 2 y 4)
- Lectura de teclado en modo raw (Video 3)
- `threading` para que el spin de ROS 2 y el bucle del teclado corran en paralelo
- `threading.Event` para bloquear el teclado mientras el robot se mueve

---

## Template

El archivo `template_node.py` contiene un esqueleto comentado con todas las secciones opcionales de un nodo (publisher, subscriber, timer, threading). Úsalo como punto de partida para crear nuevos nodos.

---

## Cómo correr los ejemplos

Los Videos 1, 2 y 4 requieren que el robot esté encendido y publicando en `/joint_states`. Lanza primero el bringup del robot:

```bash
cd ~/ros2/KIT_Phantom_X_Pincher_ROS2/phantom_ws
. install/setup.bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py
```

Luego, en otro terminal:

```bash
. install/setup.bash
ros2 run pincher_control <nombre_del_nodo>
```

El Video 3 (`ejemplo_teclado.py`) no necesita ROS 2 y se corre directamente:

```bash
python3 guias/Nodos/ejemplo_teclado.py
```

Para el teleop completo (Video 5):

```bash
. install/setup.bash
ros2 run pincher_control teleop_keyboard
```
