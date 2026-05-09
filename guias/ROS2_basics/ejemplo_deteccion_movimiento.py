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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Umbral en radianes: si la diferencia entre dos lecturas consecutivas de un
# joint supera este valor, se considera que el robot está en movimiento.
# Valor muy pequeño → sensible pero puede dar falsos positivos por ruido del sensor.
# Valor muy grande → tarda más en detectar que el robot arrancó.
UMBRAL_MOVIMIENTO = 0.01  # radianes

# Cuántas lecturas consecutivas por debajo del umbral se necesitan para
# confirmar que el robot se detuvo. Evita declarar "detuvo" por un instante
# de quietud en medio de un movimiento lento.
# A 50 Hz (frecuencia típica de /joint_states) → 10 lecturas ≈ 0.2 segundos.
LECTURAS_ESTABLES = 10


class DeteccionMovimiento(Node):

    def __init__(self):
        super().__init__('deteccion_movimiento')

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._cb_joint_states,
            10,
        )

        # Almacena las posiciones del mensaje anterior para poder comparar.
        # Empieza en None porque al recibir el primer mensaje no hay "anterior".
        self._ultimas_pos = None

        # Bandera que indica si el robot ya arrancó a moverse en el movimiento actual.
        # Se necesita porque el robot puede tardar unos instantes en arrancar después
        # de recibir un comando: sin esta bandera, detectaríamos "detuvo" antes de
        # que el robot siquiera empezara.
        self._robot_moviendose = False

        # Contador de lecturas consecutivas con delta pequeño.
        # Solo se incrementa cuando _robot_moviendose es True, es decir,
        # cuando el robot ya había arrancado y ahora parece estar quieto.
        self._lecturas_estables = 0

        self.get_logger().info(
            'Nodo listo. Mueve el robot para ver la detección en acción.'
        )

    def _cb_joint_states(self, msg):
        # En el primer mensaje no tenemos posición anterior, solo guardamos y salimos.
        if self._ultimas_pos is None:
            self._ultimas_pos = list(msg.position)
            return

        # Calculamos el delta: la mayor diferencia en radianes entre la posición
        # actual y la anterior de todos los joints.
        # Usamos el máximo (no la suma) porque basta con que UN joint se mueva
        # para saber que el robot está en movimiento.
        delta = max(
            abs(actual - anterior)
            for actual, anterior in zip(msg.position, self._ultimas_pos)
        )

        # Actualizamos la referencia para la próxima comparación
        self._ultimas_pos = list(msg.position)

        # --- Máquina de estados simple: quieto → moviendo → quieto ---

        if delta > UMBRAL_MOVIMIENTO:
            # El robot se está moviendo.
            # Si es la primera vez que detectamos movimiento en este ciclo, lo anunciamos.
            if not self._robot_moviendose:
                self.get_logger().info(f'Robot en movimiento. (delta={delta:.4f} rad)')
            self._robot_moviendose = True
            self._lecturas_estables = 0  # reiniciamos el contador de quietud

        elif self._robot_moviendose:
            # Delta pequeño Y el robot ya había arrancado → posible detención.
            # Incrementamos el contador pero no declaramos "detuvo" todavía.
            self._lecturas_estables += 1

            if self._lecturas_estables >= LECTURAS_ESTABLES:
                # Confirmado: llevan LECTURAS_ESTABLES mensajes consecutivos con
                # delta pequeño → el robot se detuvo.
                self._robot_moviendose = False
                self._lecturas_estables = 0
                self.get_logger().info('Robot detuvo. Listo para el siguiente comando.')

        # Si delta es pequeño y _robot_moviendose es False, el robot simplemente
        # está en reposo: no hacemos nada para no llenar el log de mensajes.


def main(args=None):
    rclpy.init(args=args)
    node = DeteccionMovimiento()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
