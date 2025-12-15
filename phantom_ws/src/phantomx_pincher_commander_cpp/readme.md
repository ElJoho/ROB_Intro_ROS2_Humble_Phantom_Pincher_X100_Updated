# Guía de uso: `phantomx_pincher_commander_cpp`

Este paquete proporciona nodos C++ para controlar el brazo PhantomX Pincher mediante MoveIt. Expone un nodo "commander" que escucha comandos de agarre, articulaciones y poses cartesianas, además de un binario de ejemplo "test_moveit" para probar trayectorias predefinidas.

## Dependencias principales
- ROS 2 Humble (o compatible) con soporte para `rclcpp`.
- MoveIt 2 (`moveit_ros_planning_interface`).
- Interfaces de mensajes `example_interfaces` y `phantomx_pincher_interfaces`.

Estas dependencias se declaran en `package.xml` y se vinculan en `CMakeLists.txt`, por lo que `colcon` las resolverá durante la compilación.

## Construcción del paquete
1. Ubíquese en la raíz del workspace (`phantom_ws`).
2. Compile solo este paquete:
   ```bash
   colcon build --packages-select phantomx_pincher_commander_cpp
   ```
3. Cargue las configuraciones del workspace:
   ```bash
   source install/setup.bash
   ```

## Nodos instalados
- `commander`: nodo principal que utiliza dos grupos de MoveIt (`arm` y `gripper`) para planificar y ejecutar movimientos. Sus tolerancias y tiempos de planificación se configuran al iniciar el nodo.
- `test_moveit`: ejemplo que planifica una pose objetivo y un camino cartesiano simple usando el grupo `arm`.

Ambos ejecutables se instalan en `lib/phantomx_pincher_commander_cpp/`.

## Temas que suscribe `commander`
- `/open_gripper` (`example_interfaces/Bool`): `true` abre la garra y `false` la cierra.
- `/joint_command` (`example_interfaces/Float64MultiArray`): vector de 6 articulaciones. Solo se ejecuta si el tamaño es exactamente 6.
- `/pose_command` (`phantomx_pincher_interfaces/PoseCommand`): posición (`x`, `y`, `z`), orientación en radianes (`roll`, `pitch`, `yaw`) y bandera `cartesian_path` para ejecutar un trayecto cartesiano corto en lugar de un objetivo puntual.

## Flujo de ejecución recomendado
1. Lanzar la descripción y el controlador del robot (ajuste el launch según su configuración):
   ```bash
   ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py
   ```
2. En otra terminal, iniciar el nodo de comandos:
   ```bash
   source install/setup.bash
   ros2 run phantomx_pincher_commander_cpp commander
   ```
3. Enviar comandos de prueba (cada terminal debe haber ejecutado `source install/setup.bash`):
   - Abrir la garra:
     ```bash
     ros2 topic pub -1 /open_gripper example_interfaces/Bool "{data: true}"
     ```
   - Cerrar la garra:
     ```bash
     ros2 topic pub -1 /open_gripper example_interfaces/Bool "{data: false}"
     ```
   - Mover por articulaciones (6 valores en radianes):
     ```bash
     ros2 topic pub -1 /joint_command example_interfaces/Float64MultiArray "{data: [1.2, 0.3, -0.8, 1.0, 0.0, -0.5]}"
     ```
   - Mover a una pose usando cinemática inversa:
     ```bash
     ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand        "{x: 0.128, y: 0.0, z: 0.100, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
     ```
   - Ejecutar un pequeño trayecto cartesiano desde la pose actual:
     ```bash
     ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand        "{x: 0.150, y: 0.050, z: 0.120, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: true}"
     ```

## Uso de `test_moveit`
Este binario muestra cómo planificar una pose objetivo y luego un camino cartesiano simple con el grupo `arm`. Para ejecutarlo (con el robot y MoveIt ya levantados):
```bash
ros2 run phantomx_pincher_commander_cpp test_moveit
```

## Consejos adicionales
- El frame de planificación se lee del MoveGroup de `arm`; revise el log de inicio para confirmar cuál usa su configuración.
- Si desea supervisar la pose actual del efector final, puede usar:
  ```bash
  ros2 run tf2_ros tf2_echo phantomx_pincher_base_link phantomx_pincher_end_effector
  ```