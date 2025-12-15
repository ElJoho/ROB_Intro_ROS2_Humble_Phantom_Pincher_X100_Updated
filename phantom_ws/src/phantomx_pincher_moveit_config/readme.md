# Guía de uso de `phantomx_pincher_moveit_config`

Este paquete contiene la configuración de MoveIt 2 para el brazo robótico PhantomX Pincher. Incluye los archivos de lanzamiento, configuraciones de control y descripciones semánticas necesarios para planificar trayectorias y ejecutar movimientos tanto en simulación como sobre hardware real.

## Dependencias y compilación

1. Asegúrate de tener las dependencias de ejecución instaladas (MoveIt 2, `xacro`, `robot_state_publisher`, controladores `ros2_control`, etc.). El paquete declara sus dependencias en `package.xml` y se construye con `ament_cmake`.
2. Desde el workspace (`phantom_ws`), instala dependencias y compila:
   ```bash
   rosdep install --from-paths src -y --ignore-src
   colcon build --symlink-install --packages-select phantomx_pincher_moveit_config
   source install/setup.bash
   ```

## Archivos clave

- `launch/move_group.launch.py`: lanza `move_group`, RViz y (opcionalmente) `moveit_servo` para control continuo. Incluye argumentos para seleccionar el paquete de descripción URDF/SRDF, activar la colisión, escoger el plugin de `ros2_control` (`fake`, `ign`, `real`) y habilitar RViz o Servo.
- `launch/move_group_external_control.launch.py`: pensado para usar controladores ROS 1 a través de `ros1_bridge`. Publica descripciones, `move_group` y (si se desea) `moveit_servo`, dejando la gestión de controladores externos.
- `config/*.yaml`: límites articulares, cinemática, configuración de OMPL, controladores (`controllers_position.yaml`, `controllers_effort.yaml`) y parámetros de Servo (`servo.yaml`, `servo_real.yaml`).
- `srdf/phantomx_pincher.srdf.xacro`: descripción semántica paramétrica. Puede convertirse en SRDF estático con `scripts/xacro2srdf.bash`.

## Uso básico (simulación)

1. Lanza la configuración estándar con controladores falsos y RViz:
   ```bash
   ros2 launch phantomx_pincher_moveit_config move_group.launch.py
   ```
2. Argumentos útiles (se pueden pasar por línea de comandos):
   - `ros2_control_plugin:=fake|ign|real` — selecciona el backend de control.
   - `ros2_control_command_interface:=position|velocity|effort` — el tipo de comando para los controladores.
   - `enable_servo:=true|false` — habilita el nodo `moveit_servo` para teleoperación continua.
   - `enable_rviz:=true|false` — abre RViz con la configuración incluida (`rviz/moveit.rviz`).
   - `use_sim_time:=true|false` — usa reloj simulado (útil con Gazebo/Ignition).
   Consulta todos los argumentos disponibles con:
   ```bash
   ros2 launch --show-args phantomx_pincher_moveit_config move_group.launch.py
   ```
## Actualizar el SRDF

Para regenerar el SRDF desde el xacro semántico (por ejemplo, al modificar el efector final):
```bash
cd $(ros2 pkg prefix phantomx_pincher_moveit_config)/share/phantomx_pincher_moveit_config
./scripts/xacro2srdf.bash name:=phantomx_pincher use_real_gripper:=true
```
Esto actualizará `srdf/phantomx_pincher.srdf` con los argumentos indicados.
