# Uso del paquete `phantomx_pincher_description`

Este paquete contiene la descripción del brazo robótico PhantomX Pincher en formatos URDF y SDF, junto con los recursos necesarios (mallas, configuraciones de RViz y scripts) para visualizar el modelo y reutilizarlo en otros proyectos ROS 2.

## Requisitos previos
- ROS 2 Humble (u otra versión compatible con `ament_cmake`).
- Paquetes de ejecución declarados en `package.xml`: `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `urdf`, `ros_ign_bridge` y `ros_ign_gazebo` si se desea usar Gazebo.
- Haber compilado el workspace con `colcon build` y haber ejecutado `source install/setup.bash` antes de lanzar los ejemplos.

## Estructura del paquete
- `urdf/`: xacros principales del robot (`phantomx_pincher.urdf.xacro`).
- `meshes/` y `phantomx_pincher/meshes/`: geometrías STL/DAE para colisión y visualización.
- `launch/`: lanzadores para RViz (`view.launch.py`) y Gazebo (`view_ign.launch.py`).
- `config/initial_joint_positions.yaml`: posiciones iniciales recomendadas para los articulaciones y la pinza.
- `rviz/view.rviz`: configuración lista para mostrar el modelo en RViz2.
- `scripts/`: utilidades para generar URDF (`xacro2urdf.bash`) o SDF (`xacro2sdf.bash`, `xacro2sdf_direct.bash`) a partir de los xacros.

## Generar los modelos desde xacro
Puedes obtener los archivos expandiendo los xacros directamente:

```bash
# URDF
ros2 run xacro xacro $(ros2 pkg prefix --share phantomx_pincher_description)/urdf/phantomx_pincher.urdf.xacro > /tmp/phantomx_pincher.urdf

# SDF usando el script incluido
ros2 run phantomx_pincher_description xacro2sdf.bash
```

Los scripts en `scripts/` aceptan argumentos para habilitar colisiones, prefijos de nombres o articulaciones miméticas de la pinza según lo definido en los xacros.

## Visualizar el robot en RViz2
Ejecuta el lanzador incluido para cargar la descripción en RViz2 con un panel de articulaciones interactivo:

```bash
ros2 launch phantomx_pincher_description view.launch.py
```

El archivo `view.launch.py` usa `xacro` para generar el URDF en tiempo de lanzamiento y lanza los nodos `robot_state_publisher`, `joint_state_publisher_gui` y `rviz2`. Puedes ajustar parámetros opcionales como:

- `name`: nombre base del robot (por defecto `phantomx_pincher`).
- `prefix`: prefijo de nombres para los joints y enlaces.
- `collision`: activar/desactivar la geometría de colisión.
- `mimic_finger_joints`: usar articulaciones miméticas en la pinza.
- `rviz_config`: ruta a una configuración alternativa de RViz.
- `use_sim_time`: usar reloj simulado.
- `log_level`: nivel de log para los nodos.

Ejemplo con prefijo personalizado y sin colisión:

```bash
ros2 launch phantomx_pincher_description view.launch.py prefix:=demo_ collision:=false
```

## Visualizar el robot en Gazebo
Si cuentas con Ignition/Gazebo y el bridge ROS 2 instalado, puedes abrir el modelo SDF con:

```bash
ros2 launch phantomx_pincher_description view_ign.launch.py
```

## Posiciones iniciales de las articulaciones
El archivo `config/initial_joint_positions.yaml` define las posiciones iniciales tanto del brazo como de la pinza. Úsalo como referencia o para inicializar controladores en otras aplicaciones.