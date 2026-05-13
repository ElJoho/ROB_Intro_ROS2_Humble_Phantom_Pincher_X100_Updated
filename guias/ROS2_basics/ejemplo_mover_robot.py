#!/usr/bin/env python3
"""
Ejemplo mínimo para mover el robot a posiciones predefinidas.

Publica una secuencia de nombres de estado al topic /named_target.
El nodo espera un tiempo fijo entre posiciones y se cierra al terminar.

Requiere que el robot esté encendido y el bringup lanzado:
    ros2 launch phantomx_pincher_bringup bringup.launch.py
Corre con:
    python3 ejemplo_mover_robot.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Mensaje simple de texto: solo un campo 'data' (str).

SECUENCIA = ['up', 'rest', 'ready_near', 'rest']
# Lista de nombres de estado definidos en el SRDF del robot.
# El controlador mueve el robot a cada posición cuando recibe el nombre en /named_target.

ESPERA_SEGUNDOS = 15.0
# Tiempo entre posiciones. Debe ser mayor que el tiempo que tarda el robot en llegar.
# Si el robot no alcanza a llegar antes del siguiente comando, aumentar este valor.


class MoverRobot(Node):

    def __init__(self):
        super().__init__('mover_robot')  # Nombre del nodo en la red ROS2.

        self.pub = self.create_publisher(
            String,           # Tipo de mensaje.
            '/named_target',  # Topic que escucha el controlador del robot.
            10,               # Tamaño de la cola de publicación.
        )

        self._indice = 0
        # Apunta a qué posición de SECUENCIA vamos a publicar en el siguiente tick.

        self.timer = self.create_timer(ESPERA_SEGUNDOS, self._cb_timer)
        # create_timer(periodo, callback) → ejecuta _cb_timer cada ESPERA_SEGUNDOS.
        # El primer disparo ocurre ESPERA_SEGUNDOS después de crear el timer.
        # Para enviar el primer comando inmediatamente llamamos al callback a mano:
        self._cb_timer()

    def _cb_timer(self):

        if self._indice >= len(SECUENCIA):
            # Terminamos la secuencia: cancelamos el timer y cerramos el nodo.
            self.timer.cancel()
            self.get_logger().info('Secuencia completada.')
            raise SystemExit  # rclpy.spin() captura SystemExit y sale limpiamente.

        posicion = SECUENCIA[self._indice]

        msg = String()
        msg.data = posicion  # El controlador espera solo el nombre como texto.
        self.pub.publish(msg)

        self.get_logger().info(f'[{self._indice + 1}/{len(SECUENCIA)}] Enviando: {posicion}')

        self._indice += 1


def main(args=None):
    rclpy.init(args=args)
    node = MoverRobot()
    rclpy.spin(node)       # Procesa callbacks hasta que _cb_timer lanza SystemExit.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
Ejemplo mínimo para mover el robot a posiciones predefinidas.

Publica una secuencia de nombres de estado al topic /named_target.
El nodo espera un tiempo fijo entre posiciones y se cierra al terminar.

Requiere que el robot esté encendido y el bringup lanzado:
    ros2 launch phantomx_pincher_bringup bringup.launch.py
Corre con:
    python3 ejemplo_mover_robot.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Mensaje simple de texto: solo un campo 'data' (str).

SECUENCIA = ['up', 'rest', 'ready_near', 'rest']
# Lista de nombres de estado definidos en el SRDF del robot.
# El controlador mueve el robot a cada posición cuando recibe el nombre en /named_target.

ESPERA_SEGUNDOS = 15.0
# Tiempo entre posiciones. Debe ser mayor que el tiempo que tarda el robot en llegar.
# Si el robot no alcanza a llegar antes del siguiente comando, aumentar este valor.


class MoverRobot(Node):

    def __init__(self):
        super().__init__('mover_robot')  # Nombre del nodo en la red ROS2.

        self.pub = self.create_publisher(
            String,           # Tipo de mensaje.
            '/named_target',  # Topic que escucha el controlador del robot.
            10,               # Tamaño de la cola de publicación.
        )

        self._indice = 0
        # Apunta a qué posición de SECUENCIA vamos a publicar en el siguiente tick.

        self.timer = self.create_timer(ESPERA_SEGUNDOS, self._cb_timer)
        # create_timer(periodo, callback) → ejecuta _cb_timer cada ESPERA_SEGUNDOS.
        # El primer disparo ocurre ESPERA_SEGUNDOS después de crear el timer.
        # Para enviar el primer comando inmediatamente llamamos al callback a mano:
        self._cb_timer()

    def _cb_timer(self):

        if self._indice >= len(SECUENCIA):
            # Terminamos la secuencia: cancelamos el timer y cerramos el nodo.
            self.timer.cancel()
            self.get_logger().info('Secuencia completada.')
            raise SystemExit  # rclpy.spin() captura SystemExit y sale limpiamente.

        posicion = SECUENCIA[self._indice]

        msg = String()
        msg.data = posicion  # El controlador espera solo el nombre como texto.
        self.pub.publish(msg)

        self.get_logger().info(f'[{self._indice + 1}/{len(SECUENCIA)}] Enviando: {posicion}')

        self._indice += 1


def main(args=None):
    rclpy.init(args=args)
    node = MoverRobot()
    rclpy.spin(node)       # Procesa callbacks hasta que _cb_timer lanza SystemExit.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
