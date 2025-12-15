# phantomx_pincher_bringup

Este paquete inicia el brazo PhantomX Pincher tanto en simulación con `ros2_control` como conectado al hardware real. Provee el archivo de lanzamiento principal y configuraciones de controladores que se instalan junto con el paquete.

## Contenido
- `launch/phantomx_pincher.launch.py`: lanza los nodos comunes (publicador de estado del robot y *commander* MoveIt), la pila de simulación (`ros2_control`, controladores y MoveIt con RViz) o la pila para hardware real (nodo `pincher_control` y MoveIt apuntando a ese *action server*).
- `config/controllers_position.yaml` y `config/controllers_effort.yaml`: ejemplos de parámetros para `controller_manager` con control de trayectoria para el brazo y la garra en modo de posición o de esfuerzo.
- `package.xml` y `CMakeLists.txt`: declarativos del paquete y la instalación de los recursos anteriores.

## Requisitos
- ROS 2 Humble con soporte para `colcon`.
- Dependencias en tiempo de ejecución: `phantomx_pincher_description`, `phantomx_pincher_moveit_config`, `robot_state_publisher`, `controller_manager`, `rviz2` y `pincher_control` (para el modo de hardware real). Asegúrate de resolver dependencias con `rosdep` dentro del *workspace*.

## Compilación
1. Desde la raíz del *workspace* (`phantom_ws`):
   ```bash
   colcon build --packages-select phantomx_pincher_bringup
   source install/setup.bash
   ```

## Uso
- **Simulación (predeterminado)**
  ```bash
  ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py
  ```
  Esto inicia `ros2_control` con controladores de trayectoria, `move_group` de MoveIt (sin su `ros2_control` interno) y RViz listo para interactuar.

- **Hardware real**
  ```bash
  ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=true
  ```
  En este modo no se inicia `ros2_control`. El nodo `pincher_control` expone la acción `follow_joint_trajectory` y MoveIt se conecta directamente a ella.

## Ajustes de controladores
- Para modificar ganancias o cambiar entre control por posición y por esfuerzo, edita los archivos en `config/` y actualiza la ruta usada en el lanzamiento si lo requieres. El valor actual cargado para simulación en `phantomx_pincher.launch.py` apunta al archivo `controllers_position.yaml` del paquete `phantomx_pincher_moveit_config`.
- Si utilizas tu propio hardware o simulador, adapta las interfaces de comando/estado y los nombres de articulación en las listas `joints` de los archivos YAML.

## Depuración
- Verifica que `robot_state_publisher` publique el TF del robot y que los controladores estén activos con:
  ```bash
  ros2 control list_controllers
  ```
- Si MoveIt no puede controlar el robot, revisa que el servidor de acción `follow_joint_trajectory` esté disponible (en hardware real) o que los controladores de `ros2_control` estén en estado `active` (simulación).
