# phantomx_pincher_interfaces

Paquete de interfaces ROS 2 para el manipulador Phantom X Pincher. Define el mensaje `PoseCommand` utilizado para solicitar poses o trayectorias cartesianas al manipulador.

## Estructura

- `msg/PoseCommand.msg`: mensaje con la referencia de posición y orientación.
- `CMakeLists.txt` y `package.xml`: configuración de `ament_cmake` y generación de interfaces con `rosidl_default_generators`.

### Mensaje `PoseCommand`
El mensaje contiene:

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `x` | `float64` | Posición X en metros. |
| `y` | `float64` | Posición Y en metros. |
| `z` | `float64` | Posición Z en metros. |
| `roll` | `float64` | Ángulo de rotación alrededor de X (rad). |
| `pitch` | `float64` | Ángulo de rotación alrededor de Y (rad). |
| `yaw` | `float64` | Ángulo de rotación alrededor de Z (rad). |
| `cartesian_path` | `bool` | Si es `true`, indica que la pose debe seguirse como trayectoria cartesiana. |

## Requisitos

- ROS 2 Humble.
- `ament_cmake` y `rosidl_default_generators` (se declaran en `package.xml`).

## Compilación

1. Situarse en el espacio de trabajo (`phantom_ws`).
2. Instalar dependencias: `rosdep install --from-paths src --ignore-src -r -y`.
3. Compilar las interfaces: `colcon build --packages-select phantomx_pincher_interfaces`.
4. Sourcing: `. install/setup.bash`.

## Uso

### Publicar un mensaje de prueba
```bash
ros2 topic pub /pose_command phantomx_pincher_interfaces/msg/PoseCommand "{x: 0.1, y: 0.0, z: 0.2, roll: 0.0, pitch: 0.0, yaw: 1.57, cartesian_path: true}"
```

### Crear un publicador en C++ (fragmento)
```cpp
#include "rclcpp/rclcpp.hpp"
#include "phantomx_pincher_interfaces/msg/pose_command.hpp"

auto publisher = node->create_publisher<phantomx_pincher_interfaces::msg::PoseCommand>("/pose_command", 10);
auto msg = phantomx_pincher_interfaces::msg::PoseCommand();
msg.x = 0.1; msg.y = 0.0; msg.z = 0.2;
msg.roll = 0.0; msg.pitch = 0.0; msg.yaw = 1.57;
msg.cartesian_path = true;
publisher->publish(msg);
```

### Crear un suscriptor en Python (fragmento)
```python
import rclpy
from rclpy.node import Node
from phantomx_pincher_interfaces.msg import PoseCommand

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.create_subscription(PoseCommand, '/pose_command', self.callback, 10)

    def callback(self, msg: PoseCommand):
        self.get_logger().info(f"Pose: ({msg.x}, {msg.y}, {msg.z}) yaw={msg.yaw}")

rclpy.init()
node = PoseListener()
rclpy.spin(node)
```

## Notas

- Los flags de compilación activan advertencias (`-Wall -Wextra -Wpedantic`).
- Asegúrate de hacer `source install/setup.bash` en cada terminal antes de ejecutar nodos o pruebas.