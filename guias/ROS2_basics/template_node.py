#!/usr/bin/env python3

# rclpy es la librería principal de ROS2 para Python
import rclpy

# Node es la clase base de la que hereda todo nodo ROS2
from rclpy.node import Node

# Importar los tipos de mensaje que vayas a usar, por ejemplo:
# from std_msgs.msg import String          # mensaje de texto simple
# from sensor_msgs.msg import JointState   # estado de los joints del robot
# import threading   # si necesitas un hilo secundario (ej. lectura de teclado)


# Todo nodo ROS2 en Python es una clase que hereda de Node
class MyCustomNode(Node):  # MODIFICAR NOMBRE

    def __init__(self):
        # Inicializa el nodo con el nombre que verán otros nodos en la red ROS2
        super().__init__('node_name')  # MODIFICAR NOMBRE

        # --- PUBLISHER (opcional) ---
        # Registra este nodo como publicador en un tópico.
        # Argumentos: tipo de mensaje, nombre del tópico, tamaño de cola
        # self.pub = self.create_publisher(TipoMensaje, '/nombre_topico', 10)

        # --- SUBSCRIBER (opcional) ---
        # Registra este nodo como suscriptor de un tópico.
        # Argumentos: tipo de mensaje, nombre del tópico, callback, tamaño de cola
        # self.sub = self.create_subscription(
        #     TipoMensaje, '/nombre_topico', self._callback, 10)

        # --- TIMER (opcional) ---
        # Ejecuta self._timer_callback de forma periódica cada N segundos
        # self.timer = self.create_timer(1.0, self._timer_callback)

        # get_logger().info() imprime un mensaje en la consola con nivel INFO
        self.get_logger().info('Nodo iniciado.')

    # Callback del timer: se ejecuta automáticamente cada vez que el timer dispara
    # def _timer_callback(self):
    #     msg = TipoMensaje()   # crear instancia del mensaje
    #     msg.data = ...        # llenar los campos del mensaje
    #     self.pub.publish(msg) # publicar en el tópico

    # Callback del subscriber: se ejecuta automáticamente al recibir un mensaje
    # def _callback(self, msg):
    #     self.get_logger().info(f'Recibido: {msg.data}')


def main(args=None):
    # Inicializa la comunicación con el sistema ROS2 (debe llamarse antes de crear nodos)
    rclpy.init(args=args)

    # Crea una instancia del nodo
    node = MyCustomNode()  # MODIFICAR NOMBRE

    # --- OPCIÓN CON THREADING (si necesitas un hilo bloqueante, ej. teclado) ---
    # spin() se mueve a un hilo secundario para no bloquear el hilo principal
    # import threading
    # hilo = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # hilo.start()
    # tu_funcion_bloqueante(node)   # ej. lectura de teclado en modo raw

    # spin() mantiene el nodo vivo y ejecuta los callbacks (timer, suscripciones, etc.)
    # Bloquea hasta que se interrumpa con Ctrl+C
    rclpy.spin(node)

    # Limpieza al salir
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
