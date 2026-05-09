# ROS 2 - parte 1: Nodos publisher y subscribers


**Autor:** Johan Alejandro López Arias ([@ElJoho](https://github.com/ElJoho))


Workspace, paquetes, publisher y subscriber con el ejemplo de `amorsito`.

---
 
## Videos del tutorial
 
<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=j1m0AffoNac">Capítulo 1: Introducción a ROS2 básico</a>
  </summary>
</details>
<br>
<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=KZQwFtwZieU">Capítulo 2: Nodos, workspaces y paquetes</a>
  </summary>
</details>
<br>
<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=l2Yy4gGGv5A">Capítulo 3: Creando mi primer publisher de ROS2</a>
  </summary>
</details>
<br>
<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=gwu9l_zS1Qc">Capítulo 4: Creando mi primer subscriber de ROS2</a>
  </summary>
</details>
<br>

---
 
## Índice
 
1. [Conceptos clave](#1-conceptos-clave)
2. [Workspace y paquetes](#2-workspace-y-paquetes)
   - 2.1. [Crear el workspace](#21-crear-el-workspace)
   - 2.2. [Crear los paquetes](#22-crear-los-paquetes)
   - 2.3. [Estructura de los paquetes](#23-estructura-de-los-paquetes)
3. [Publisher en Python](#3-publisher-en-python)
4. [Subscriber en Python](#4-subscriber-en-python)
5. [Compilar y correr](#5-compilar-y-correr)
6. [Comandos útiles para inspeccionar el grafo](#6-comandos-útiles-para-inspeccionar-el-grafo)
---
 
## 1. Conceptos clave
 
- **Nodo**: proceso ejecutable que realiza una tarea específica dentro del sistema. Es la unidad básica de cómputo en ROS 2.
- **Workspace**: directorio de trabajo donde se construyen y organizan los paquetes mediante `colcon`.
- **Paquete**: unidad mínima de organización y distribución de código en ROS 2. Contiene nodos, mensajes, configuraciones y archivos de build.
- **Tópico**: canal nombrado de comunicación asíncrona basado en el patrón publish/subscribe, tipado por una interfaz de mensaje específica.
- **Mensaje**: estructura de datos tipada que se transmite a través de un tópico.
- **Publisher**: entidad de un nodo que envía mensajes a un tópico.
- **Subscriber**: entidad de un nodo que recibe mensajes de un tópico mediante un callback.
Un mismo nodo puede contener publishers y subscribers simultáneamente.
 
---
 
## 2. Workspace y paquetes
 
### 2.1. Crear el workspace
 
```bash
mkdir amorsito_ws
cd amorsito_ws
mkdir src
cd ..
colcon build
```
 
> **Importante**: `colcon build` siempre se ejecuta en la **raíz del workspace** (la carpeta que contiene `src/`). Si lo corres dentro de `src/` se generan `build/`, `install/` y `log/` en el lugar equivocado y rompe el workspace.
 
Si pasa eso, se limpia con:
 
```bash
rm -rf build install log
```
 
Después de `colcon build` el workspace queda así:
 
```
amorsito_ws/
├── build/
├── install/
├── log/
└── src/
```
 
### 2.2. Crear los paquetes
 
Dentro de `src/`:
 
```bash
cd src
# Paquete en Python
ros2 pkg create --build-type ament_python amorsito_py_pkg --dependencies rclpy
 
# Paquete en C++
ros2 pkg create --build-type ament_cmake amorsito_cpp_pkg --dependencies rclcpp
```
 
- `rclpy` = ROS Client Library para Python.
- `rclcpp` = ROS Client Library para C++.
### 2.3. Estructura de los paquetes
 
**Paquete Python (`amorsito_py_pkg`)**:
```
amorsito_py_pkg/
├── amorsito_py_pkg/      <- aquí van los scripts .py de los nodos
├── package.xml           <- dependencias del paquete
├── setup.py              <- aquí se registran los nodos como ejecutables
└── setup.cfg
```
 
**Paquete C++ (`amorsito_cpp_pkg`)**:
```
amorsito_cpp_pkg/
├── include/
├── src/                  <- aquí van los .cpp de los nodos
├── package.xml           <- dependencias del paquete
└── CMakeLists.txt        <- aquí se registran los ejecutables
```
 
---
 
## 3. Publisher en Python
 
Archivo: `amorsito_py_pkg/amorsito_py_pkg/amorsito_publisher.py`
 
```python
#!/usr/bin/env python3
# Shebang: indica al sistema que este script se ejecuta con Python 3.
# Necesario para que ROS2 lo ejecute como nodo.

import rclpy  # Librería principal de ROS2 para Python (client library).
from rclpy.node import Node  # Clase base Node, de la cual hereda todo nodo de ROS2.
from std_msgs.msg import String  # Tipo de mensaje estándar String, que vamos a publicar.


# ESTADOS = ['up','rest','ready_near','ready_mid']
# Lista original (comentada) que probablemente sería para un brazo robótico.

ESTADOS = ['te_amo', 'te_odio', 'te_ignoro', 'NO_IGNORES_QUE_TE_IGNORO']
# Lista de strings que el nodo va a publicar de forma cíclica.
# Es una constante global, por convención en MAYÚSCULAS.


class AmorsitoPublisher(Node):
    # Clase del nodo publisher, hereda de Node para acceder a la funcionalidad de ROS2.
    
    def __init__(self):
        # Constructor: se ejecuta al instanciar el nodo.
        
        super().__init__('amorsito_publisher')
        # Llama al constructor de Node y registra este nodo en el grafo de ROS2
        # con el nombre 'amorsito_publisher'. Es el nombre que verás con 'ros2 node list'.
        
        self.pub = self.create_publisher(
            String,                  # Tipo de mensaje que vamos a publicar.
            '/topico_de_amorsito',   # Nombre del tópico donde se publica.
            10                       # Tamaño de la cola: cuántos mensajes se almacenan en buffer
                                     # si los suscriptores procesan más lento de lo que publicamos.
        )
        
        self._indice = 0
        # Contador interno que llevará el índice del estado actual.
        # El guion bajo al inicio es convención de Python para indicar "atributo privado".
        
        self.timer = self.create_timer(3.0, self._timer_callback)
        # Crea un timer que ejecuta '_timer_callback' cada 3.0 segundos.
        # ROS2 se encarga de llamarlo automáticamente mientras el nodo esté vivo.
        
        self.get_logger().info('Amorsito publisher esta listo, se publica en /topico_de_amorsito cada 3 s.')
        # Mensaje en el log para confirmar que el nodo arrancó bien.

    def _timer_callback(self):
        # Callback del timer: se ejecuta automáticamente cada 3 segundos.
        
        estado = ESTADOS[self._indice % len(ESTADOS)]
        # Selecciona un estado de la lista de forma cíclica.
        # El operador módulo (%) hace que cuando el índice supere el tamaño de la lista,
        # vuelva a empezar desde 0 sin tirar IndexError.
        
        self._indice += 1
        # Incrementa el índice para que la próxima vez se publique el siguiente estado.
        
        msg = String()
        # Crea una instancia vacía del mensaje String.
        
        msg.data = estado
        # Asigna el string seleccionado al campo 'data' del mensaje.
        # std_msgs/String tiene un único campo 'data' de tipo string.
        
        self.pub.publish(msg)
        # Publica el mensaje en el tópico /topico_de_amorsito.
        # Cualquier nodo suscrito recibirá este mensaje.
        
        self.get_logger().info(f'Publicado: {msg.data}')
        # Imprime en el log lo que acabamos de publicar (útil para debug).


def main(args=None):
    # Punto de entrada del programa, llamado por 'ros2 run' a través del setup.py.
    
    rclpy.init(args=args)
    # Inicializa el cliente de ROS2. Debe ejecutarse antes de crear cualquier nodo.
    
    node = AmorsitoPublisher()
    # Crea la instancia del nodo (dispara el __init__ de la clase).
    
    rclpy.spin(node)
    # Mantiene el nodo vivo procesando callbacks (en este caso, el del timer).
    # Bloquea aquí hasta que se cierre el nodo (Ctrl+C).
    
    node.destroy_node()
    # Libera los recursos del nodo de forma ordenada (publishers, timers, etc.).
    
    rclpy.shutdown()
    # Cierra el cliente de ROS2. Contraparte de rclpy.init().


if __name__ == '__main__':
    # Solo se ejecuta si el archivo se corre directamente, no si se importa.
    main()
```
 
Pasos clave del patrón:
1. `rclpy.init()` antes de crear cualquier nodo.
2. Heredar de `Node` y registrar el nombre con `super().__init__('nombre_nodo')`.
3. `create_publisher(tipo, tópico, cola)`.
4. `create_timer(periodo, callback)` para publicar periódicamente.
5. `rclpy.spin(node)` para que ROS 2 procese los callbacks.
---
 
## 4. Subscriber en Python
 
Archivo: `amorsito_py_pkg/amorsito_py_pkg/amorsito_subscriber.py`
 
```python
#!/usr/bin/env python3
# Shebang: indica al sistema que este script se ejecuta con Python 3.
# Necesario para que ROS2 pueda ejecutarlo como un nodo.

import rclpy  # Librería principal de ROS2 para Python (cliente library).
from rclpy.node import Node  # Clase base Node, de la cual hereda todo nodo de ROS2.
from std_msgs.msg import String  # Tipo de mensaje estándar String, el mismo que usa el publisher.


class EjemploSubscriber(Node):
    # Definimos nuestra clase de nodo, heredando de Node para tener acceso
    # a toda la funcionalidad de ROS2 (loggers, suscripciones, timers, etc.).
    
    def __init__(self):
        # Constructor del nodo: se ejecuta al instanciar la clase.
        
        super().__init__('amorsito_subscriber')
        # Llama al constructor de la clase padre (Node) y le pasa el nombre
        # con el que este nodo aparecerá en el grafo de ROS2.
        # Es el nombre que verás con 'ros2 node list'.
        
        self.subscriber_ = self.create_subscription(
            String,                       # Tipo de mensaje que vamos a recibir.
            '/topico_de_amorsito',        # Nombre del tópico al que nos suscribimos
                                          # (debe coincidir exactamente con el del publisher).
            self._subscriber_callback_,   # Función que se ejecuta cada vez que llega un mensaje.
            10                            # Tamaño de la cola: cuántos mensajes se guardan en buffer
                                          # si llegan más rápido de lo que podemos procesarlos.
        )
        
        self.get_logger().info('El nodo amorsito_subscriber ha comenzado a stalkear el topico de amorsito.')
        # Mensaje informativo en el log para confirmar que el nodo arrancó correctamente.
        
    def _subscriber_callback_(self, msg: String):
        # Callback: ROS2 lo invoca automáticamente cuando llega un mensaje al tópico.
        # 'msg' es el mensaje recibido, del tipo String que declaramos arriba.
        # El type hint ': String' es opcional pero ayuda al editor con el autocompletado.
        
        self.get_logger().info(msg.data)
        # Imprime el contenido del campo 'data' del mensaje en el log.
        # Recuerda: std_msgs/String tiene un solo campo llamado 'data' que es un string.


def main(args=None):
    # Punto de entrada del programa, requerido por ROS2 para ejecutar el nodo
    # vía 'ros2 run' (apunta a esta función desde el setup.py).
    
    rclpy.init(args=args)
    # Inicializa el cliente de ROS2. Debe llamarse antes de crear cualquier nodo.
    
    node = EjemploSubscriber()
    # Crea una instancia de nuestro nodo (ejecuta el __init__ de arriba).
    
    rclpy.spin(node)
    # Mantiene el nodo "vivo" procesando callbacks indefinidamente.
    # Esta línea bloquea aquí hasta que el nodo se cierre (Ctrl+C).
    
    node.destroy_node()
    # Limpia los recursos del nodo de forma ordenada (suscripciones, timers, etc.).
    
    rclpy.shutdown()
    # Cierra el cliente de ROS2. Contraparte de rclpy.init().


if __name__ == "__main__":
    # Esto solo se ejecuta si el archivo se corre directamente,
    # no si se importa desde otro módulo.
    main()
```
 
Diferencia central con el publisher: en lugar de `create_publisher` + timer, se usa `create_subscription` con un callback que ROS 2 invoca cada vez que llega un mensaje al tópico.
 
---
 
## 5. Compilar y correr
 
Desde la raíz del workspace:
 
```bash
cd ~/amorsito_ws
colcon build
source install/setup.bash
 
# En una terminal
ros2 run amorsito_py_pkg amorsito_publisher
 
# En otra terminal (recordá hacer source de nuevo)
source install/setup.bash
ros2 run amorsito_py_pkg amorsito_subscriber
```
 
> Para que `ros2 run` encuentre los nodos, hay que registrarlos como `entry_points` en el `setup.py` del paquete Python.
 
---
 
## 6. Comandos útiles para inspeccionar el grafo
 
```bash
ros2 node list                          # Lista los nodos activos
ros2 topic list                         # Lista los tópicos activos
ros2 topic echo /topico_de_amorsito     # Imprime los mensajes del tópico
```
 
Publicar un mensaje manual desde la terminal (útil para probar el subscriber sin levantar el publisher):
 
```bash
ros2 topic pub -1 /topico_de_amorsito std_msgs/msg/String "{data: 'dime_lindura'}"
```
 
El flag `-1` publica el mensaje una sola vez y termina.
