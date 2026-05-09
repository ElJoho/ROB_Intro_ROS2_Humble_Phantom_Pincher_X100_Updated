import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import threading
import sys
import tty
import termios


# Mapeo de teclas a estados nombrados del SRDF
TECLAS = {
    'a': 'up',
    'b': 'rest',
    'c': 'ready_near',
    'd': 'ready_mid',
}

UMBRAL_MOVIMIENTO = 0.01  # radianes — delta mínimo para considerar que el robot se mueve
LECTURAS_ESTABLES = 10    # lecturas consecutivas sin movimiento para considerar que el robot paró
                          # a 50 Hz esto equivale a ~0.2 segundos de quietud


class TeleopPincher(Node):

    def __init__(self):
        super().__init__('teleop_pincher')

        self.pub = self.create_publisher(String, '/named_target', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 10)

        # Event para bloquear el hilo del teclado mientras el robot se mueve.
        # ocupado.set()   → robot en movimiento, ignorar teclas
        # ocupado.clear() → robot parado, aceptar teclas
        self._ocupado = threading.Event()

        self._lock = threading.Lock()
        self._ultimas_pos = None      # últimas posiciones de joints recibidas
        self._robot_moviendose = False  # True cuando el robot ya arrancó a moverse
        self._lecturas_estables = 0   # contador de lecturas consecutivas sin movimiento

        self.get_logger().info(
            'Teleop listo.\n'
            '  a → up\n'
            '  b → rest\n'
            '  c → ready_near\n'
            '  d → ready_mid\n'
            '  q → salir'
        )

    def publicar(self, estado):
        """Publica un estado nombrado y marca el robot como ocupado."""
        with self._lock:
            self._robot_moviendose = False
            self._lecturas_estables = 0
        self._ocupado.set()

        msg = String()
        msg.data = estado
        self.pub.publish(msg)
        self.get_logger().info(f'Publicado: {estado}')

    def _cb_joint_states(self, msg):
        """
        Callback de /joint_states.
        Compara posiciones entre lecturas para detectar si el robot
        arrancó a moverse y cuándo se detuvo.
        """
        # Si no hay movimiento en curso, solo actualizar posiciones de referencia
        if not self._ocupado.is_set():
            with self._lock:
                self._ultimas_pos = list(msg.position)
            return

        with self._lock:
            if self._ultimas_pos is None:
                self._ultimas_pos = list(msg.position)
                return

            delta = max(
                abs(actual - anterior)
                for actual, anterior in zip(msg.position, self._ultimas_pos)
            )
            self._ultimas_pos = list(msg.position)

            if delta > UMBRAL_MOVIMIENTO:
                # El robot está en movimiento
                self._robot_moviendose = True
                self._lecturas_estables = 0
            elif self._robot_moviendose:
                # El robot ya se había movido y ahora parece estar quieto
                self._lecturas_estables += 1
                if self._lecturas_estables >= LECTURAS_ESTABLES:
                    # Confirmado: el robot terminó de moverse
                    self._robot_moviendose = False
                    self._lecturas_estables = 0
                    self._ocupado.clear()
                    self.get_logger().info('Robot detuvo. Listo para nueva tecla.')


def leer_teclado(node):
    """
    Bucle principal del teclado. Corre en el hilo principal.
    Pone la terminal en modo raw para leer teclas sin necesitar Enter.
    Restaura la terminal al salir.
    """
    fd = sys.stdin.fileno()
    config_original = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            tecla = sys.stdin.read(1)

            if tecla == 'q':
                node.get_logger().info('Saliendo...')
                rclpy.shutdown()
                break

            if node._ocupado.is_set():
                node.get_logger().info(f'Robot ocupado, ignorando tecla: {tecla!r}')
                continue

            if tecla in TECLAS:
                node.publicar(TECLAS[tecla])
            else:
                node.get_logger().info(f'Tecla desconocida: {tecla!r}')

    finally:
        # IMPORTANTE: siempre restaurar la terminal, incluso si hay error
        termios.tcsetattr(fd, termios.TCSADRAIN, config_original)


def main():
    rclpy.init()
    node = TeleopPincher()

    # rclpy.spin corre en un hilo separado para no bloquear el teclado
    hilo_spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    hilo_spin.start()

    try:
        leer_teclado(node)
    except Exception as e:
        node.get_logger().error(f'Error inesperado: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
