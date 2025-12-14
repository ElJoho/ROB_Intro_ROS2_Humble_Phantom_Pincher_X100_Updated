# Importar las librerías
# Esta es OpenCV
import cv2

# Estos son módulos y librerías de paquetes de ROS 2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

# El argumento "Node" significa que la clase PhantomxPincherCameraPublisher hereda
# (o es hija de) la clase llamada Node.
# La clase Node es una clase estándar de ROS 2.
class PhantomxPincherCameraPublisher(Node):


    # Constructor
    def __init__(self):

        # Esta función se usa para inicializar los atributos de la clase padre
        super().__init__('pxp_camera_publisher_node')

        # Aquí creamos una instancia del objeto OpenCV VideoCapture
        # Este es el número del dispositivo de la cámara – debes ajustar correctamente este número
        # dependiendo del número de dispositivo de la cámara asignado por el sistema Linux
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

        # CvBridge se usa para convertir imágenes de OpenCV a mensajes de ROS 2
        # que pueden enviarse a través de los tópicos
        self.bridgeObject = CvBridge()

        # Nombre del tópico usado para transferir las imágenes de la cámara
        # Este nombre de tópico debe coincidir con el nombre del tópico en el nodo suscriptor
        self.topicNameFrames = 'phantomx_pincher_camera'

        # El tamaño de la cola para los mensajes
        self.queueSize = 20

        # Aquí, la función "self.create_publisher" crea el publicador que
        # publica mensajes del tipo Image, sobre el tópico self.topicNameFrames
        # y con self.queueSize
        self.publisher = self.create_publisher(
            Image,
            self.topicNameFrames,
            self.queueSize
        )

        # Período de comunicación en segundos
        self.periodCommunication = 0.02

        # Crear el temporizador que llama a la función self.timer_callbackFunction
        # cada self.periodCommunication segundos
        self.timer = self.create_timer(
            self.periodCommunication,
            self.timer_callbackFunction
        )

        # Este es el contador que registra cuántas imágenes se publican
        self.i = 0

    # Esta es la función callback que se llama cada self.periodCommunication segundos
    def timer_callbackFunction(self):

        # Aquí leemos el frame usando la cámara
        success, frame = self.camera.read()

        # Redimensionar la imagen
        frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)

        # Si logramos leer el frame
        if success == True:

            # Aquí convertimos el frame de OpenCV a un mensaje Image de ROS 2
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame)

            # Publicar la imagen
            self.publisher.publish(ROS2ImageMessage)

            # Usar el logger para mostrar el mensaje en la pantalla
            self.get_logger().info(
                'Publicando imagen número %d' % self.i
            )

            # Actualizar el contador de imágenes
            self.i += 1

# Esta es la función principal y es el punto de entrada de nuestro código
def main(args=None):

    # Inicializar rclpy
    rclpy.init(args=args)

    # Crear el objeto publicador
    cameraPublisherNode = PhantomxPincherCameraPublisher()

    # Aquí hacemos spin, y la función callback del temporizador se llama recursivamente
    rclpy.spin(cameraPublisherNode)

    # Destruir
    cameraPublisherNode.destroy_node()

    # Apagar
    rclpy.shutdown()


if __name__ == '__main__':
    main()
