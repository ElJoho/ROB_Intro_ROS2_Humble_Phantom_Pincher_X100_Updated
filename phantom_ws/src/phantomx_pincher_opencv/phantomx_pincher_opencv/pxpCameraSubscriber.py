# Importar las librerías
# Esta es OpenCV
import cv2

# Estos son módulos y librerías de paquetes de ROS 2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

# El argumento "Node" significa que la clase PhantomxPincherCameraSubscriber hereda
# (o es hija de) la clase llamada Node.
# La clase Node es una clase estándar de ROS 2

class PhantomxPincherCameraSubscriber(Node):

    # Constructor
    def __init__(self):

        # Esta función se usa para inicializar los atributos de la clase padre
        super().__init__('pxp_camera_subscriber_node')

        # CvBridge se usa para convertir imágenes de OpenCV a mensajes de ROS 2
        # que pueden enviarse a través de los tópicos
        self.bridgeObject = CvBridge()

        # Nombre del tópico usado para transferir las imágenes de la cámara
        # Este nombre de tópico debe coincidir con el nombre del tópico en el nodo publicador
        self.topicNameFrames = 'phantomx_pincher_camera'

        # El tamaño de la cola para los mensajes
        self.queueSize = 20

        # Aquí, la función "self.create_subscription" crea el suscriptor que
        # se suscribe a mensajes del tipo Image, sobre el tópico self.topicNameFrames
        # y con self.queueSize
        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callbackFunction,
            self.queueSize
        )
        self.subscription # Esto se usa para evitar advertencias de variable sin uso

    # Esta es una función callback que muestra la imagen recibida
    def listener_callbackFunction(self, imageMessage):

        # Mostrar el mensaje en la consola
        self.get_logger().info('Se recibió el frame de imagen')

        # Convertir el mensaje de imagen de ROS 2 a imagen de OpenCV
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        # Mostrar la imagen en la pantalla
        cv2.imshow("Video de la cámara", openCVImage)
        cv2.waitKey(1)

# Esta es la función principal y es el punto de entrada de nuestro código
def main(args=None):

    # Inicializar rclpy
    rclpy.init(args=args)

    # Crear el objeto suscriptor
    cameraSubscriberNode = PhantomxPincherCameraSubscriber()

    # Aquí hacemos spin, y la función callback se llama recursivamente
    rclpy.spin(cameraSubscriberNode)

    # Destruir
    cameraSubscriberNode.destroy_node()

    # Apagar
    rclpy.shutdown()


if __name__ == '__main__':
    main()
