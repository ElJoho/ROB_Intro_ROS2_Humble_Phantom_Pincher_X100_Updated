# Parámetro `use_suction_cup`: activar y desactivar la ventosa

## Descripción general

El modelo URDF del brazo PhantomX Pincher incluye una ventosa (suction cup) como herramienta final opcional. Para evitar tener que comentar y descomentar código manualmente, se añadió el parámetro `use_suction_cup` que permite activar o desactivar la ventosa en tiempo de lanzamiento o al regenerar el SRDF.

**Por defecto la ventosa está desactivada (`use_suction_cup:=false`).** Para usarla hay que especificarlo explícitamente en el comando de lanzamiento.

---

## Qué se activa y qué no

### Elementos controlados por `use_suction_cup`

Cuando el parámetro es `false` (valor por defecto), estos dos elementos **no aparecen en el modelo**:

| Elemento | Tipo | Padre |
|---|---|---|
| `{prefix}suction_cup_link` | Link (visual + colisión + inercia) | — |
| `{prefix}suction_cup_joint` | Joint fijo | `{prefix}coupling_tool_end_effector_link` |

Además, en el SRDF de MoveIt, todas las reglas de `disable_collisions` que referencian `{prefix}suction_cup_link` también se omiten automáticamente.

### Elementos que **siempre están presentes** (no afectados)

El acople que une el gripper con la ventosa **nunca se desactiva**:

| Elemento | Tipo | Descripción |
|---|---|---|
| `{prefix}coupling_tool_end_effector_link` | Link | Pieza de acople física entre el gripper y la ventosa |
| `{prefix}acople_suction_cup_joint` | Joint fijo | Conecta `gripper_servo_link` con el acople |

Sus reglas de colisión en el SRDF tampoco se ven afectadas.

---

## Archivos modificados

### `phantomx_pincher_description/urdf/phantomx_pincher_arm.xacro`

Se añadió `use_suction_cup:=false` a los parámetros del macro `phantomx_pincher`:

```xml
<xacro:macro name="phantomx_pincher" params="
  prefix
  ...
  use_suction_cup:=false
">
```

El link y el joint de la ventosa están envueltos en un bloque condicional:

```xml
<xacro:if value="${use_suction_cup}">
  <link name="${prefix}suction_cup_link"> ... </link>
  <joint name="${prefix}suction_cup_joint" type="fixed"> ... </joint>
</xacro:if>
```

---

### `phantomx_pincher_description/urdf/phantomx_pincher.urdf.xacro`

Se declaró el argumento de entrada con valor por defecto `false`:

```xml
<xacro:arg name="use_suction_cup" default="false"/>
```

Y se pasa al macro del brazo:

```xml
<xacro:phantomx_pincher
    ...
    use_suction_cup="$(arg use_suction_cup)"/>
```

---

### `phantomx_pincher_moveit_config/srdf/phantomx_pincher.xacro`

Se añadió `use_suction_cup:=false` a los parámetros del macro SRDF:

```xml
<xacro:macro name="phantomx_pincher" params="prefix:=${prefix} use_real_gripper:=true use_suction_cup:=false">
```

Las reglas de colisión de la ventosa (sección C) están envueltas en un bloque condicional. Las del acople (sección B) permanecen sin condición:

```xml
<!-- B) Colisiones del acople — siempre presentes -->
<disable_collisions link1="${prefix}coupling_tool_end_effector_link" ... />
...

<!-- C) Colisiones de la ventosa — solo cuando use_suction_cup=true -->
<xacro:if value="${use_suction_cup}">
  <disable_collisions link1="${prefix}suction_cup_link" ... />
  ...
</xacro:if>
```

---

### `phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf.xacro`

Se declaró el argumento con default `false` y se pasa al macro:

```xml
<xacro:arg name="use_suction_cup" default="false" />

<xacro:phantomx_pincher
    prefix="$(arg prefix)"
    use_real_gripper="$(arg use_real_gripper)"
    use_suction_cup="$(arg use_suction_cup)" />
```

---

### `phantomx_pincher_moveit_config/launch/move_group.launch.py`

Se declaró el argumento de lanzamiento con default `false`:

```python
DeclareLaunchArgument(
    "use_suction_cup",
    default_value="false",
    description="Flag to enable the suction cup link and joint.",
),
```

Se recoge el valor y se pasa a los comandos xacro del URDF y del SRDF:

```python
use_suction_cup = LaunchConfiguration("use_suction_cup")

# En el comando xacro del URDF:
"use_suction_cup:=", use_suction_cup,

# En el comando xacro del SRDF:
"use_suction_cup:=", use_suction_cup,
```

---

### `phantomx_pincher_bringup/launch/phantomx_pincher.launch.py`

Se declaró el argumento con default `false`:

```python
use_suction_cup = LaunchConfiguration("use_suction_cup")

DeclareLaunchArgument(
    "use_suction_cup",
    default_value="false",
    description="If 'true', include the suction cup link and joint in the robot model.",
),
```

El URDF ahora recibe el argumento en el comando xacro:

```python
robot_description = ParameterValue(
    Command(["xacro ", urdf_path, " use_suction_cup:=", use_suction_cup]),
    value_type=str,
)
```

El SRDF fue cambiado de leer el archivo estático con `cat` a procesarlo con `xacro` en tiempo de ejecución. Se debe pasar `name:=phantomx_pincher` explícitamente porque el archivo `.srdf.xacro` usa `$(arg name)` en el atributo `<robot name="...">` antes de que el argumento sea declarado, lo que causa un error si no se pasa:

```python
# ANTES (no funcionaba con el parámetro):
Command(["cat ", srdf_path])   # srdf_path → phantomx_pincher.srdf

# AHORA:
Command(["xacro ", srdf_path, " name:=phantomx_pincher use_suction_cup:=", use_suction_cup])
# srdf_path → phantomx_pincher.srdf.xacro
```

Ambos includes de `move_group.launch.py` reciben el argumento:

```python
launch_arguments={
    ...
    "use_suction_cup": use_suction_cup,
}.items()
```

---

## Cómo usarlo

### Con `move_group.launch.py`

Sin ventosa (por defecto):
```bash
ros2 launch phantomx_pincher_moveit_config move_group.launch.py
```

Con ventosa:
```bash
ros2 launch phantomx_pincher_moveit_config move_group.launch.py use_suction_cup:=true
```

### Con `phantomx_pincher.launch.py` (bringup)

Sin ventosa (por defecto):
```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py
```

Con ventosa:
```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_suction_cup:=true
```

Se pueden combinar múltiples argumentos:
```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=true use_suction_cup:=true
```

### Para inspeccionar el URDF con xacro directamente

Se debe usar la ruta completa al archivo:

```bash
# Sin ventosa (por defecto):
xacro ~/ros2/KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_description/urdf/phantomx_pincher.urdf.xacro

# Con ventosa:
xacro ~/ros2/KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_description/urdf/phantomx_pincher.urdf.xacro use_suction_cup:=true
```

### Para regenerar el SRDF estático

Ejecutar desde la carpeta del paquete `phantomx_pincher_moveit_config`:

```bash
# Sin ventosa (por defecto):
bash scripts/xacro2srdf.bash

# Con ventosa:
bash scripts/xacro2srdf.bash use_suction_cup:=true
```

### Tras cualquier cambio, hacer build

```bash
colcon build --packages-select phantomx_pincher_description phantomx_pincher_moveit_config phantomx_pincher_bringup
source install/setup.bash
```

---

## Nota sobre el segfault de move_group al cerrar

Al hacer Ctrl+C, `move_group` puede mostrar un segmentation fault. Este error ocurre en el destructor interno de MoveIt2 (`~MoveItCpp`, `~TrajectoryExecutionManager`) y es un bug conocido de MoveIt2 en ROS2 Jazzy. **No afecta al funcionamiento durante la ejecución** y no está relacionado con el parámetro `use_suction_cup`.

---

## Jerarquía de links con ventosa activa (`use_suction_cup:=true`)

```
gripper_servo_link
└── acople_suction_cup_joint (fijo)
    └── coupling_tool_end_effector_link     ← siempre presente
        └── suction_cup_joint (fijo)
            └── suction_cup_link            ← solo si use_suction_cup:=true
```

## Jerarquía de links sin ventosa / por defecto (`use_suction_cup:=false`)

```
gripper_servo_link
└── acople_suction_cup_joint (fijo)
    └── coupling_tool_end_effector_link     ← siempre presente
```
