# phantomx_pincher_opencv

Paquete ROS 2 (Python) para **publicar** imágenes desde una webcam usando OpenCV y **suscribirse** para visualizar el video y detectar un **cuadrado negro** (por ejemplo, una pieza cuadrada negra sobre fondo claro).

---

## 1) ¿Qué hace este paquete?

Incluye dos nodos:

- **Publicador**: lee frames desde la cámara (OpenCV `VideoCapture`) y publica mensajes `sensor_msgs/Image` en el tópico `phantomx_pincher_camera`.
- **Suscriptor**: se suscribe al mismo tópico, muestra el video con `cv2.imshow(...)` y aplica detección de **cuadrado negro** (bordes + validación de interior oscuro).

> Nota: La detección está pensada para “cuadrado negro visto en cámara” (cara frontal de un cubo o una lámina cuadrada), y puede requerir ajustes si hay brillos, sombras o perspectiva.

---

## 2) Requisitos

- Ubuntu 24.04 (o similar) con entorno gráfico (para ver ventanas de OpenCV).
- ROS 2 Jazzy (o distro compatible).
- Cámara USB reconocida por Linux (ej: Logitech C270).

### Dependencias típicas (Ubuntu/ROS 2 Jazzy)

```bash
sudo apt update
sudo apt install python3-opencv
sudo apt install ros-jazzy-cv-bridge ros-jazzy-sensor-msgs
```

Opcional (para visualizar imágenes sin tu suscriptor):

```bash
sudo apt install ros-jazzy-image-view ros-jazzy-rqt-image-view
```

---

## 3) Estructura y entry points

Este paquete se llama **`phantomx_pincher_opencv`** y define dos ejecutables ROS 2 en `setup.py`:

- `pxp_camera_publisher_node`
- `pxp_camera_subscriber_node`

(Se ejecutan con `ros2 run phantomx_pincher_opencv <ejecutable>`.)

---

## 4) Instalación en un workspace (colcon)

1) Copia/coloca la carpeta del paquete dentro de `src/` de tu workspace, por ejemplo:

```bash
mkdir -p ~/phantom_ws/src
# Copia aquí la carpeta phantomx_pincher_opencv/
```

2) Compila el workspace:

```bash
cd ~/phantom_ws
colcon build --symlink-install
```

3) “Source” del workspace (en **cada terminal** que vayas a usar):

```bash
source /opt/ros/jazzy/setup.bash
source ~/phantom_ws/install/setup.bash
```

---

## 5) Uso básico

### Opción A: Publicador + Suscriptor (con detección)

**Terminal 1 (publicador):**
```bash
ros2 run phantomx_pincher_opencv pxp_camera_publisher_node
```

**Terminal 2 (suscriptor con detección y overlay):**
```bash
ros2 run phantomx_pincher_opencv pxp_camera_subscriber_node
```

Se abrirá una ventana de OpenCV con el título:
- `Video de la cámara (detección de cuadrado negro)`

Si detecta el cuadrado, dibuja un rectángulo rotado verde y un texto con métricas.

### Opción B: Publicador + visor de ROS (sin detección)

**Terminal 1 (publicador):**
```bash
ros2 run phantomx_pincher_opencv pxp_camera_publisher_node
```

**Terminal 2 (visor):**
```bash
ros2 run image_view image_view --ros-args -r image:=/phantomx_pincher_camera
```

> En `image:=...` **NO** va una ruta de archivos. Va el **nombre del tópico**.

---

## 6) Verificar que todo está “vivo”

Ver tópicos:

```bash
ros2 topic list
```

Verificar tipo del tópico:

```bash
ros2 topic info /phantomx_pincher_camera
```

Ver FPS aproximados:

```bash
ros2 topic hz /phantomx_pincher_camera
```

---

## 7) Ajustes útiles

### 7.1 Cambiar el dispositivo de cámara

Si tu cámara no está en `/dev/video0`, edita **`pxpCameraPublisher.py`** y cambia:

- `self.cameraDeviceNumber = 0`  → prueba `1`, `2`, etc.

### 7.2 Reducir la tasa de publicación (FPS)

Edita **`pxpCameraPublisher.py`**:

- `self.periodCommunication = 0.02`  → ~50 FPS objetivo
- Ejemplos:
  - **10 FPS**: `self.periodCommunication = 0.1`
  - **5 FPS**: `self.periodCommunication = 0.2`

Luego recompila:

```bash
cd ~/phantom_ws
colcon build --symlink-install
source ~/phantom_ws/install/setup.bash
```

### 7.3 Ajustar la detección del cuadrado negro

Edita **`pxpCameraSubscriber.py`** (parámetros típicos):

- `self.maxMeanGray`: sube este valor si el negro sale “gris” por iluminación (ej. 70–90).
- `self.darkPixelThreshold`: umbral de píxel oscuro (ej. 80–110).
- `self.minDarkFraction`: porcentaje mínimo de píxeles oscuros dentro del candidato.
- `self.minRectArea`: sube si detecta ruido; baja si el cuadrado está lejos.
- `self.cannyLow / self.cannyHigh`: si los bordes salen débiles o muy ruidosos.

#### Ventanas de depuración

Para ver bordes (Canny) activa:

- `self.showDebugWindows = True`

Esto abre una ventana adicional: `DEBUG - Edges`.

---

## 8) Consejos y solución de problemas

### No aparece ninguna ventana de OpenCV
- Estás en un entorno **sin GUI** (headless) o sin acceso a display.
- Verifica:

```bash
echo $DISPLAY
```

- Si usas NoMachine/Remote Desktop, asegúrate de que la sesión sea gráfica y tenga acceso a la pantalla.

### `image_view` “Package not found”
Instala el visor:

```bash
sudo apt update
sudo apt install ros-jazzy-image-view
```

### Mucho “spam” en consola (logs)
Ambos nodos imprimen logs por frame. Si te baja el rendimiento, comenta el `get_logger().info(...)` en los scripts.

---

## 9) Archivos principales

- `setup.py` (define el paquete y los ejecutables ROS 2)
- `pxpCameraPublisher.py` (publicador de cámara)
- `pxpCameraSubscriber.py` (suscriptor + detección de cuadrado negro)

---

## 10) Créditos / base
Este paquete se inspira en el flujo típico de publicador/suscriptor de cámara con OpenCV en ROS 2 (ver también el documento de referencia `ros_2_camera_publisher_subscriber_readme.md` incluido en el proyecto).