#!/usr/bin/env python3
"""
Ejemplo de detección de movimiento del robot.

Extiende el ejemplo_subscriber.py añadiendo la lógica para determinar
si el robot está en movimiento o se detuvo, comparando posiciones
consecutivas de /joint_states.

Sin publisher, sin teclado, sin threading — solo la lógica de detección.

Requiere que el robot esté encendido y publicando en /joint_states.
Corre con:
    ros2 run pincher_control ejemplo_deteccion_movimiento
    (o directamente: python3 ejemplo_deteccion_movimiento.py)
"""

import rclpy  # Cliente Python de ROS2: init/shutdown del runtime y spin de nodos.
from rclpy.node import Node  # Clase base de la que hereda todo nodo ROS2 en Python.
from sensor_msgs.msg import JointState  # Mensaje estándar para el estado de las articulaciones.
# Campos relevantes de JointState:
#   name[]     → nombres de los joints (ej: 'joint_1', 'joint_2', ...)
#   position[] → posiciones angulares (rad) o lineales (m) de cada joint
#   velocity[] → velocidades de cada joint (rad/s o m/s)
#   effort[]   → torque/fuerza aplicada en cada joint


UMBRAL_VELOCIDAD = 0.005   # rad/s — usado cuando msg.velocity tiene valores reales.
# Mide velocidad directamente, así que es independiente de la frecuencia del topic.
# 0.005 rad/s ≈ 0.5 % de la velocidad típica máxima del Dynamixel → detecta arranques muy lentos.
# Valor agresivo: si aparecen falsos positivos con el robot quieto, subir gradualmente.

UMBRAL_MOVIMIENTO = 0.0005  # rad entre frames — fallback cuando velocity no está disponible.
# A 50 Hz, 0.0005 rad/frame corresponde a ~0.025 rad/s.
# Se bajó desde 0.01 (valor original) para detectar movimientos muy lentos.
# El costo: ruido de encoder mayor a 0.0005 rad puede dar falsos positivos.
# Verificar con el robot quieto que la oscilación de las posiciones esté por debajo de este umbral.

LECTURAS_ESTABLES = 25
# Cuántos frames seguidos por debajo del umbral confirman que el robot se detuvo.
# A 50 Hz, 25 frames ≈ 0.5 s. Se aumentó desde 15 porque al bajar los umbrales
# el sistema queda más sensible al ruido y los frames "casi quietos" se vuelven
# más frecuentes; necesitamos más evidencia consecutiva para confirmar la detención.


class DeteccionMovimiento(Node):

    def __init__(self):
        super().__init__('deteccion_movimiento')  # Registra el nodo en ROS2 con este nombre.

        self.sub = self.create_subscription(
            JointState,              # Tipo de mensaje que vamos a recibir.
            '/joint_states',         # Topic estándar donde se publica el estado del robot.
            self._cb_joint_states,   # Callback que se ejecuta por cada mensaje recibido.
            10,                      # Tamaño de la cola (QoS depth): hasta 10 mensajes en buffer.
        )

        self._ultimas_pos = None
        # Guarda las posiciones del frame anterior para compararlas con el actual.
        # None al inicio porque en el primer mensaje aún no hay "anterior" con qué comparar.

        self._robot_moviendose = False
        # Bandera de estado: ¿el robot ya arrancó el movimiento actual?
        # Importante porque entre que enviamos un comando y el robot reacciona pasan unos ms;
        # sin esta bandera podríamos declarar "se detuvo" antes incluso de que empiece a moverse.

        self._lecturas_estables = 0
        # Cuenta cuántos frames consecutivos llevamos con delta pequeño.
        # Solo se incrementa cuando _robot_moviendose es True (es decir,
        # cuando estamos esperando a confirmar una detención, no en reposo permanente).

        self.get_logger().info(
            'Nodo listo. Mueve el robot para ver la detección en acción.'
        )  # Log informativo para confirmar que el nodo arrancó correctamente.

    def _cb_joint_states(self, msg):

        tiene_velocidad = bool(msg.velocity) and any(v != 0.0 for v in msg.velocity)
        # ¿El firmware publica velocidades reales en este mensaje?
        # bool(msg.velocity) → False si la lista llega vacía.
        # any(v != 0.0 ...) → False si todos los valores son cero, lo que indica
        # que el firmware no reporta velocidad (no es que el robot esté quieto).

        if tiene_velocidad:
            # === Opción B: usar velocidad directamente (preferida) ===

            delta = max(abs(v) for v in msg.velocity)
            # Tomamos la velocidad más alta entre todos los joints (en valor absoluto).
            # Basta con que UN joint se mueva para considerar al robot en movimiento.

            umbral = UMBRAL_VELOCIDAD  # Comparamos contra el umbral en rad/s.

            self._ultimas_pos = list(msg.position)
            # Aunque ahora no usemos posiciones, las guardamos por si el firmware
            # deja de reportar velocidad en algún frame futuro y caemos al fallback.

        else:
            # === Opción A (fallback): comparar posiciones entre frames ===

            if self._ultimas_pos is None:
                self._ultimas_pos = list(msg.position)  # Primer mensaje: solo guardamos.
                return  # Sin "anterior" no hay nada que comparar, salimos del callback.

            delta = max(
                abs(actual - anterior)
                for actual, anterior in zip(msg.position, self._ultimas_pos)
            )
            # zip() empareja cada joint con su valor anterior y calculamos la mayor
            # diferencia absoluta. Como antes, basta con que UN joint se mueva.

            umbral = UMBRAL_MOVIMIENTO  # Comparamos contra el umbral en radianes.

            self._ultimas_pos = list(msg.position)
            # Actualizamos para el siguiente frame.
            # list() crea una copia mutable propia — no compartimos referencia
            # con el buffer interno del mensaje, que ROS puede reutilizar.

        # --- Máquina de estados: REPOSO → MOVIENDO → REPOSO ---

        if delta > umbral:
            # El robot se está moviendo en este frame.

            if not self._robot_moviendose:
                # Transición REPOSO → MOVIENDO: anunciamos solo en el primer frame
                # del movimiento para no llenar el log de mensajes repetidos.
                unidad = 'rad/s' if tiene_velocidad else 'rad'
                self.get_logger().info(
                    f'Robot en movimiento. (delta={delta:.4f} {unidad})'
                )

            self._robot_moviendose = True   # Confirmamos/mantenemos el estado MOVIENDO.
            self._lecturas_estables = 0     # Reiniciamos: cualquier movimiento rompe la racha de quietud.

        elif self._robot_moviendose:
            # Delta pequeño Y veníamos moviéndonos → posible detención en curso.

            self._lecturas_estables += 1
            # Sumamos un frame estable, pero todavía NO declaramos detención;
            # un solo frame quieto podría ser ruido o un cambio de dirección.

            if self._lecturas_estables >= LECTURAS_ESTABLES:
                # Acumulamos suficiente evidencia: el robot realmente se detuvo.
                self._robot_moviendose = False  # Transición MOVIENDO → REPOSO.
                self._lecturas_estables = 0     # Reiniciamos el contador para el próximo ciclo.
                self.get_logger().info('Robot detuvo. Listo para el siguiente comando.')

        # Caso implícito: delta pequeño Y _robot_moviendose == False → el robot está
        # en reposo permanente. No hacemos nada (evita inundar el log).


def main(args=None):
    rclpy.init(args=args)              # Inicializa el runtime de ROS2.
    node = DeteccionMovimiento()       # Instancia nuestro nodo.
    rclpy.spin(node)                   # Bucle principal: procesa callbacks hasta Ctrl+C.
    node.destroy_node()                # Libera recursos del nodo (suscripciones, timers...).
    rclpy.shutdown()                   # Cierra el runtime de ROS2 limpiamente.


if __name__ == '__main__':
    main()
