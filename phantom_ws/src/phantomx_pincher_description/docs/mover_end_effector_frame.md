# Cómo mover el frame del End Effector (TCP)

## ¿Qué es el `end_effector`?

Es un **link virtual** — no tiene geometría visual ni de colisión, solo existe como
frame en el árbol TF de ROS 2. MoveIt lo usa como TCP (Tool Center Point) para calcular
la cinemática inversa: "¿qué ángulos necesitan las articulaciones para que *este punto*
llegue a *esta pose*?"

Lo genera el macro `phantomx_pincher_virtual_link` en `phantomx_pincher_utils.xacro`,
que internamente llama a `phantomx_pincher_dummy_link` (link sin `<visual>`,
sin `<collision>`, sin `<inertial>` real).

En RViz aparece únicamente como un eje de coordenadas XYZ, nunca como malla 3D.

---

## Jerarquía de links relevantes

```
gripper_finger_base_link
    └──[end_effector_virtual_joint]──> end_effector   ← TCP de MoveIt (aquí se mueve)
            └──[acople_suction_cup_joint]──> coupling_tool_end_effector_link
                    └──[suction_cup_joint]──> suction_cup_link
```

Todo lo que está debajo de `end_effector` (el acople y la ventosa) se mueve
automáticamente al mover este frame, porque son joints fijos encadenados.

---

## ¿Por qué el RPY es `(π/2, -π/2, π/2)`?

El `gripper_finger_base_link` llega al gripper con su eje Z apuntando "hacia arriba"
en la geometría local, producto de la cadena de brackets y servos.

MoveIt exige que el **eje Z del end effector apunte en la dirección de aproximación**
(hacia el objeto a agarrar). La rotación `(roll=90°, pitch=-90°, yaw=90°)` corrige esa
discrepancia:

| Eje del `end_effector` después de la rotación | Significado para MoveIt |
|---|---|
| **Z** | Dirección de aproximación (entre los dedos, hacia el objeto) |
| **X** | Dirección de deslizamiento lateral |
| **Y** | Normal al plano de agarre |

Sin esta rotación, la cinemática inversa calcularía el acercamiento en la
dirección incorrecta.

---

## Parámetros que controlan la posición/orientación del TCP

En `phantomx_pincher_arm.xacro` (líneas ~433-439):

```xml
<xacro:phantomx_pincher_virtual_link
    parent="${prefix}gripper_finger_base_link"
    link_name="${prefix}end_effector"
    joint_origin_xyz="${finger_centre_offset_x} 0.0 0.0"   <!-- ← POSICIÓN -->
    joint_origin_rpy="${pi/2} ${-pi/2} ${pi/2}"             <!-- ← ORIENTACIÓN -->
    gazebo_preserve_fixed_joint="${gazebo_preserve_fixed_joint}"
    />
```

- `joint_origin_xyz` → traslación XYZ en metros, relativa a `gripper_finger_base_link`
- `joint_origin_rpy` → orientación en radianes (roll, pitch, yaw), relativa a `gripper_finger_base_link`

El valor inicial de X viene de la propiedad:

```xml
<xacro:property name="finger_centre_offset_x" value="0.019"/>  <!-- 19 mm -->
```

---

## Pasos para trasladar el TCP

### Paso 1 — Identificar dónde quieres el TCP

Mide (en metros) la distancia desde `gripper_finger_base_link` hasta el nuevo
punto de control deseado, por ejemplo la punta de la ventosa.

Puedes obtener la distancia actual con:

```bash
ros2 run tf2_ros tf2_echo gripper_finger_base_link end_effector
```

Y la distancia hasta la ventosa con:

```bash
ros2 run tf2_ros tf2_echo gripper_finger_base_link suction_cup_link
```

### Paso 2 — Editar `joint_origin_xyz`

Abre `phantomx_pincher_arm.xacro` y modifica el valor de la traslación:

```xml
<!-- Ejemplo: adelantar el TCP 5 cm más en X -->
joint_origin_xyz="${finger_centre_offset_x + 0.05} 0.0 0.0"

<!-- O con valor fijo -->
joint_origin_xyz="0.069 0.0 0.0"
```

Los tres valores son `X Y Z` en metros respecto a `gripper_finger_base_link`:

| Componente | Efecto |
|---|---|
| Aumentar X | Acerca el TCP hacia la punta de los dedos / ventosa |
| Cambiar Y | Desplaza lateralmente (entre los dedos es 0) |
| Cambiar Z | Sube/baja el TCP |

### Paso 3 — Ajustar la orientación si es necesario

Si el nuevo tool tiene una orientación de aproximación distinta, cambia `joint_origin_rpy`:

```xml
<!-- Girar 90° adicionales sobre Z -->
joint_origin_rpy="${pi/2} ${-pi/2} ${pi}"
```

> **Nota:** Cambiar el RPY afecta cómo MoveIt calcula la orientación de la pose
> objetivo. Si solo trasladas el punto (misma dirección de aproximación), no es
> necesario cambiar el RPY.

### Paso 4 — Recompilar y verificar en RViz

```bash
cd ~/ros2/KIT_Phantom_X_Pincher_ROS2/phantom_ws
colcon build --packages-select phantomx_pincher_description
source install/setup.bash
```

Lanza RViz y activa la visualización de TF. Verifica que el frame `end_effector`
aparece en la posición correcta respecto a la ventosa:

```bash
ros2 launch phantomx_pincher_description display.launch.py
```

En el panel TF de RViz activa `end_effector` y comprueba que el eje Z (azul)
apunta hacia el objeto a agarrar.

### Paso 5 — Verificar que MoveIt sigue usando el frame correcto

Comprueba el archivo SRDF para confirmar que el grupo de planificación apunta
a `end_effector` como tip link:

```bash
grep -r "end_effector" \
  ~/ros2/KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_moveit_config/srdf/
```

Si cambiaste el nombre del link o añadiste uno nuevo, actualiza el SRDF en
`phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf` o `.xacro`.

---

## Ejemplo completo: TCP en la punta de la ventosa

Si la punta de la ventosa está a ~90 mm del `gripper_finger_base_link` en X:

```xml
<xacro:phantomx_pincher_virtual_link
    parent="${prefix}gripper_finger_base_link"
    link_name="${prefix}end_effector"
    joint_origin_xyz="0.090 0.0 0.0"
    joint_origin_rpy="${pi/2} ${-pi/2} ${pi/2}"
    gazebo_preserve_fixed_joint="${gazebo_preserve_fixed_joint}"
    />
```

> Usa `ros2 run tf2_ros tf2_echo` para medir la distancia exacta
> antes de hardcodear el valor.
