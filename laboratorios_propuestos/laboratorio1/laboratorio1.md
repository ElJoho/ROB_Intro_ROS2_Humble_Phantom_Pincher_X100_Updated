# Práctica de Laboratorio: Control del PhantomX Pincher con MoveIt + Publisher/Subscriber

## Objetivos

Al finalizar esta práctica el estudiante será capaz de:

1. Definir y mover el robot **PhantomX Pincher** a 4 poses distintas usando MoveIt en ROS2.
2. Implementar un nodo **publisher** y un nodo **subscriber** en ROS2 que se comuniquen mediante mensajes de texto.
3. modificar archivos **XACRO/URDF** del robot para identificar visualmente la unidad y al equipo de trabajo.

---

## Requisitos previos

- Tener instalado ROS2 (Humble o superior) y MoveIt2.
- Tener clonado y compilado el repositorio del kit:
  [`labsir-un/KIT_Phantom_X_Pincher_ROS2`](https://github.com/labsir-un/KIT_Phantom_X_Pincher_ROS2)
- Conocimientos básicos de Python, XML y la línea de comandos.
- Haber leído la guía de comandos de movimiento:
  [`guias/Moveit/MOTION_COMMANDS.md`](https://github.com/labsir-un/KIT_Phantom_X_Pincher_ROS2/blob/main/guias/Moveit/MOTION_COMMANDS.md)

---

## Parte 1 — ROS2: Mover el robot con un Publisher/Subscriber

### 1.1 Definir 4 poses del robot

Cada grupo debe definir **4 poses distintas** para el PhantomX Pincher. Las poses pueden expresarse como:

- **Valores articulares (joint values)** — un arreglo con la posición de cada articulación.
- **Pose cartesiana** — posición `(x, y, z)` y orientación del efector final.

> Apóyese en la guía `MOTION_COMMANDS.md` del repositorio para ver la sintaxis exacta de los comandos `ros2 topic pub` y los formatos de mensaje aceptados.

Sugerencia de poses (ejemplo, ajuste según su grupo):

| Letra | Nombre de la pose | Descripción |
|-------|-------------------|-------------|
| `a`   | `home`            | Posición de reposo, brazo vertical |
| `b`   | `pick`            | Posición de agarre sobre la mesa |
| `c`   | `place`           | Posición de depósito |
| `d`   | `rest`            | Posición plegada |

### 1.2 Probar las poses manualmente

Antes de programar el nodo, verifique que cada pose funciona enviándola desde la terminal con `ros2 topic pub` (vea la guía de comandos del repositorio para la estructura del mensaje).

```bash
# Lanzar MoveIt con el robot
ros2 launch phantom_x_pincher_moveit demo.launch.py

# En otra terminal, publicar una pose de prueba
ros2 topic pub /<topic_de_pose> <tipo_de_msg> "{...datos de la pose...}"
```

Asegúrese de que las 4 poses son alcanzables (sin colisiones ni singularidades) **antes** de pasar al siguiente paso.

### 1.3 Crear el nodo Subscriber (escucha el comando)

Cree un paquete nuevo en su workspace:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python practica_pubsub --dependencies rclpy std_msgs geometry_msgs
```

Dentro de `practica_pubsub/practica_pubsub/`, cree un archivo `pose_commander.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose  # o el tipo que corresponda a su pose

class PoseCommander(Node):
    def __init__(self):
        super().__init__('pose_commander')

        # Diccionario letra -> pose (defina las 4 poses de su grupo)
        self.poses = {
            'a': self._make_pose_home(),
            'b': self._make_pose_pick(),
            'c': self._make_pose_place(),
            'd': self._make_pose_rest(),
        }

        # Subscriber: escucha letras en el topic /comando_pose
        self.subscription = self.create_subscription(
            String,
            '/comando_pose',
            self.callback_comando,
            10
        )

        # Publisher: publica la pose objetivo al topic que MoveIt/el controlador escucha
        self.publisher = self.create_publisher(
            Pose,
            '/pose_objetivo',   # ajuste al topic real de su configuración
            10
        )

        self.get_logger().info('Nodo PoseCommander listo. Esperando letras a/b/c/d...')

    def callback_comando(self, msg: String):
        letra = msg.data.strip().lower()
        if letra in self.poses:
            pose_msg = self.poses[letra]
            self.publisher.publish(pose_msg)
            self.get_logger().info(f'Letra "{letra}" recibida -> pose enviada.')
        else:
            self.get_logger().warn(f'Letra "{letra}" no reconocida. Use a, b, c o d.')

    # --- Definición de las 4 poses ---
    def _make_pose_home(self):
        p = Pose()
        # TODO: rellene con los valores de su pose 'home'
        return p

    def _make_pose_pick(self):
        p = Pose()
        # TODO
        return p

    def _make_pose_place(self):
        p = Pose()
        # TODO
        return p

    def _make_pose_rest(self):
        p = Pose()
        # TODO
        return p


def main(args=None):
    rclpy.init(args=args)
    node = PoseCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> **Nota:** El nodo es a la vez **subscriber** (escucha letras) y **publisher** (envía la pose). Puede dividirlo en dos nodos separados si lo prefiere, pero un solo nodo con ambos roles es totalmente válido en ROS2.

### 1.4 Registrar el ejecutable

Edite `setup.py` del paquete y añada en `entry_points`:

```python
entry_points={
    'console_scripts': [
        'pose_commander = practica_pubsub.pose_commander:main',
    ],
},
```

Compile el workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select practica_pubsub
source install/setup.bash
```

### 1.5 Ejecutar y probar

En distintas terminales:

```bash
# Terminal 1: lanzar MoveIt
ros2 launch phantom_x_pincher_moveit demo.launch.py

# Terminal 2: lanzar el nodo
ros2 run practica_pubsub pose_commander

# Terminal 3: enviar comandos
ros2 topic pub --once /comando_pose std_msgs/msg/String "data: 'a'"
ros2 topic pub --once /comando_pose std_msgs/msg/String "data: 'b'"
ros2 topic pub --once /comando_pose std_msgs/msg/String "data: 'c'"
ros2 topic pub --once /comando_pose std_msgs/msg/String "data: 'd'"
```

El robot debe moverse a la pose correspondiente con cada letra.

---

## Parte 2 — URDF/XACRO: Personalizar el robot

El objetivo es modificar el archivo XACRO del PhantomX Pincher para que cada robot quede **identificado visualmente** y muestre el nombre del equipo.

### 2.1 Cambios requeridos

Su grupo debe añadir al XACRO:

1. **Número del robot** visible en:
   - La **base** del robot.
   - La **bomba de vacío** (suction cup gripper).
2. Un **elemento visual en la caja** que muestre el número del robot.
3. Un **elemento visual en la caja** con los **nombres de los integrantes** del grupo.

### 2.2 Sugerencias de implementación

Hay varias formas de añadir estos elementos visuales en XACRO. Elija la que prefiera:

#### Opción A — Texturas/imágenes en una mesh

1. Cree una imagen PNG/JPG con el número del robot o los nombres del grupo.
2. Aplíquela como textura a un `<visual>` plano usando una mesh `.dae` con material UV-mapeado.

```xml
<link name="placa_identificacion">
  <visual>
    <origin xyz="0 0 0.001" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://mi_paquete/meshes/placa_grupo.dae"/>
    </geometry>
  </visual>
</link>
```

#### Opción B — Geometrías simples con materiales de color

Si no quiere lidiar con texturas, puede usar cajas (`<box>`) o cilindros (`<cylinder>`) con colores distintivos para cada robot.

```xml
<link name="marca_numero_robot">
  <visual>
    <origin xyz="0 0 0.005" rpy="0 0 0"/>
    <geometry>
      <box size="0.04 0.04 0.002"/>
    </geometry>
    <material name="rojo_robot_3">
      <color rgba="0.8 0.1 0.1 1.0"/>
    </material>
  </visual>
</link>
```

#### Opción C — Parámetros XACRO para el número de robot

Para que el archivo sea reutilizable, defina el número como un parámetro:

```xml
<xacro:macro name="placa_robot" params="numero parent_link">
  <link name="placa_${numero}">
    <visual>
      <geometry>
        <mesh filename="package://mi_paquete/meshes/numero_${numero}.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="placa_${numero}_joint" type="fixed">
    <parent link="${parent_link}"/>
    <child link="placa_${numero}"/>
    <origin xyz="0 0 0.01" rpy="0 0 0"/>
  </joint>
</xacro:macro>

<!-- Usar la macro -->
<xacro:placa_robot numero="3" parent_link="base_link"/>
<xacro:placa_robot numero="3" parent_link="suction_cup_link"/>
```

### 2.3 Visualizar en RViz

```bash
ros2 launch phantom_x_pincher_description display.launch.py
```

Verifique que:
- El número aparece en la base y en la bomba de vacío.
- La caja muestra el número del robot.
- La caja muestra los nombres del equipo.

---

## Entregables

Cada grupo debe entregar:

1. **Repositorio** (o carpeta comprimida) con:
   - El paquete `practica_pubsub` completo.
   - El XACRO modificado con las identificaciones visuales.
   - Las meshes/imágenes usadas para la personalización.
2. **Video corto (1–2 min)** mostrando:
   - El robot moviéndose a las 4 poses al recibir las letras a/b/c/d.
   - Vista en RViz con las marcas de identificación visibles.
3. **Informe breve** (máximo 3 páginas) con:
   - Descripción de las 4 poses elegidas y por qué.
   - Capturas de pantalla del nodo funcionando y de RViz.
   - Dificultades encontradas y cómo se resolvieron.

---

## Rúbrica de evaluación

| Criterio | Peso |
|----------|------|
| Las 4 poses funcionan correctamente | 25% |
| El nodo publisher/subscriber responde a las 4 letras | 25% |
| Personalización del XACRO (número en base + bomba) | 20% |
| Identificación de la caja (número + nombres) | 15% |
| Calidad del informe y video | 15% |

---

## Recursos útiles

- Guía de comandos de movimiento: `guias/Moveit/MOTION_COMMANDS.md`
- Documentación de ROS2 sobre [Publishers y Subscribers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- Documentación de [XACRO](http://wiki.ros.org/xacro)
- Documentación de [MoveIt2](https://moveit.picknik.ai/main/index.html)
