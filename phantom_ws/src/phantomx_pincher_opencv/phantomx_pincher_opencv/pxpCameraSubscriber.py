# Importar las librerías
# Esta es OpenCV
import cv2
import numpy as np

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
        self.bridgeObject = CvBridge()

        # Nombre del tópico usado para transferir las imágenes de la cámara
        # Este nombre de tópico debe coincidir con el nombre del tópico en el nodo publicador
        self.topicNameFrames = 'phantomx_pincher_camera'

        # El tamaño de la cola para los mensajes
        self.queueSize = 20

        # -------------------------
        # Parámetros de detección
        # -------------------------
        # Umbrales para Canny (bordes)
        self.cannyLow = 40
        self.cannyHigh = 120

        # Dilatación para cerrar huecos en los bordes
        self.dilateIterations = 1

        # Área mínima del rectángulo rotado (w*h) para considerar un candidato
        # Si detecta mucho ruido, súbelo (por ejemplo 5000 o 10000)
        self.minRectArea = 2000

        # El cuadrado puede estar rotado; esto valida que sea "casi cuadrado"
        # 1.0 = cuadrado perfecto, 1.25 tolera algo de perspectiva
        self.maxAspectRatio = 1.25

        # Validación 1: el interior del candidato debe ser "oscuro"
        # Si tu cuadrado negro no es tan negro por iluminación, sube este valor (ej. 70 u 80)
        self.maxMeanGray = 60

        # Validación 2: un porcentaje mínimo de píxeles dentro del candidato debe ser oscuro
        # Esto ayuda a evitar falsos positivos con objetos con brillos o bordes oscuros
        self.darkPixelThreshold = 80
        self.minDarkFraction = 0.60

        # Activar ventanas de depuración
        self.showDebugWindows = True

        # Aquí, la función "self.create_subscription" crea el suscriptor
        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callbackFunction,
            self.queueSize
        )
        self.subscription  # Esto se usa para evitar advertencias de variable sin uso

    def _find_black_square(self, bgrImage):
        """Encuentra el mejor candidato de cuadrado negro en la imagen.
        Retorna (boxPoints, meanGray, darkFraction, aspectRatio, rectArea) o None.
        """

        # Convertir a escala de grises y suavizar para reducir ruido
        gray = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detectar bordes
        edges = cv2.Canny(blur, self.cannyLow, self.cannyHigh)

        # Cerrar pequeños huecos en los bordes
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.dilate(edges, kernel, iterations=self.dilateIterations)

        # Encontrar contornos externos
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        bestScore = -1

        for c in contours:
            # Rectángulo rotado alrededor del contorno
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), angle = rect

            if w <= 1 or h <= 1:
                continue

            rectArea = float(w * h)
            if rectArea < self.minRectArea:
                continue

            aspectRatio = max(w, h) / min(w, h)
            if aspectRatio > self.maxAspectRatio:
                continue

            # Caja del rectángulo rotado (4 puntos)
            box = cv2.boxPoints(rect)
            box = np.int32(box)

            # Crear máscara del rectángulo para medir brillo promedio dentro
            mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.fillConvexPoly(mask, box, 255)

            # Extraer píxeles dentro del candidato
            pixels = gray[mask == 255]
            if pixels.size == 0:
                continue

            meanGray = float(pixels.mean())
            darkFraction = float((pixels < self.darkPixelThreshold).mean())

            # Debe ser oscuro por dentro (dos validaciones)
            if meanGray > self.maxMeanGray:
                continue
            if darkFraction < self.minDarkFraction:
                continue

            # Puntaje: preferimos el candidato más grande
            score = rectArea
            if score > bestScore:
                bestScore = score
                best = (box, meanGray, darkFraction, aspectRatio, rectArea)

        if self.showDebugWindows:
            cv2.imshow("DEBUG - Edges", edges)

        return best

    # Esta es una función callback que se ejecuta cuando llega un frame
    def listener_callbackFunction(self, imageMessage):

        # Mostrar el mensaje en la consola (puedes comentar esto si es demasiado spam)
        self.get_logger().info('Se recibió el frame de imagen')

        # Convertir el mensaje de imagen de ROS 2 a imagen de OpenCV
        # Intentamos forzar bgr8 para que OpenCV trabaje consistente
        try:
            openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')
        except Exception:
            openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        # Asegurar que sea BGR (3 canales)
        if len(openCVImage.shape) == 2:
            openCVImage = cv2.cvtColor(openCVImage, cv2.COLOR_GRAY2BGR)

        # Buscar el cuadrado negro
        detection = self._find_black_square(openCVImage)

        # Dibujar resultados
        output = openCVImage.copy()
        if detection is not None:
            box, meanGray, darkFraction, aspectRatio, rectArea = detection

            # Dibujar el rectángulo rotado detectado
            cv2.polylines(output, [box], True, (0, 255, 0), 2)

            # Poner etiqueta (incluye métricas útiles para ajuste)
            x, y = box[0][0], box[0][1]
            label = f"Cuadrado negro | mean={meanGray:.1f} dark={darkFraction:.2f} ar={aspectRatio:.2f}"
            cv2.putText(
                output,
                label,
                (max(0, x - 20), max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA
            )

        # Mostrar la imagen en la pantalla
        cv2.imshow("Video de la cámara (detección de cuadrado negro)", output)
        cv2.waitKey(1)


# Esta es la función principal y es el punto de entrada de nuestro código
def main(args=None):

    # Inicializar
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