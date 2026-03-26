# Añadir un link de ventosa al efector final

Este documento explica cómo añadir un link fijo de ventosa al efector final del brazo PhantomX Pincher, incluyendo geometría de colisión y la actualización del objetivo de cinemática inversa (TCP) de MoveIt a la punta de la ventosa.

## Resumen

Se deben modificar tres archivos:

| Archivo | Propósito |
|---------|-----------|
| `phantomx_pincher_description/urdf/phantomx_pincher_arm.xacro` | Añadir el nuevo link y joints al URDF |
| `phantomx_pincher_moveit_config/srdf/phantomx_pincher.xacro` | Actualizar el tip de la cadena IK en la plantilla SRDF |
| `phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf` | Aplicar los mismos cambios con nombres de links fijos |

## Archivos de malla (mesh)

- **Visual**: `phantomx_pincher_description/meshes/DAE/acopleSuctionCupGripper.dae`
- **Colisión**: `phantomx_pincher_description/meshes/STL/acopleSuctionCupGripper.stl`

---

## Archivo 1: `phantomx_pincher_arm.xacro`

Localiza el bloque del link virtual `end_effector` (alrededor de la línea 362) y añade lo siguiente **después** de él.

### A) Link de la ventosa con geometría visual y de colisión

```xml
<!-- Adaptador de ventosa montado en el efector final -->
<link name="${prefix}suction_cup_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://phantomx_pincher_description/meshes/DAE/acopleSuctionCupGripper.dae"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://phantomx_pincher_description/meshes/STL/acopleSuctionCupGripper.stl"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="${prefix}end_effector_to_suction_cup_joint" type="fixed">
  <parent link="${prefix}end_effector"/>
  <child link="${prefix}suction_cup_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

> **Nota sobre la inercia**: Los valores son aproximaciones provisionales. Actualiza `mass` y el tensor de inercia cuando conozcas las propiedades físicas reales de la ventosa.

> **Nota sobre el offset**: El origen del joint es actualmente `0 0 0`. Una vez que conozcas el desplazamiento físico desde el frame del efector final hasta la base del adaptador de ventosa, actualiza el valor `xyz` en consecuencia.

### B) Link virtual TCP en la punta de la ventosa

Este es un link vacío (sin geometría) que marca la punta de la ventosa. MoveIt lo usará como objetivo para la cinemática inversa. El macro `phantomx_pincher_virtual_link` ya está definido en `phantomx_pincher_utils.xacro`.

```xml
<!-- Frame TCP virtual en la punta de la ventosa -->
<xacro:phantomx_pincher_virtual_link
    parent="${prefix}suction_cup_link"
    link_name="${prefix}suction_cup_tip"
    joint_origin_xyz="0.0 0.0 0.0"
    joint_origin_rpy="0.0 0.0 0.0"
    gazebo_preserve_fixed_joint="${gazebo_preserve_fixed_joint}"
    />
```

> **Nota sobre el offset de la punta**: El `joint_origin_xyz` es `0 0 0` como valor provisional. Una vez que midas la longitud física de la ventosa, ajusta el valor `z` a la distancia desde la base de la ventosa hasta su punta, de modo que el TCP quede exactamente en el punto de contacto.

---

## Archivo 2: `phantomx_pincher.xacro` (plantilla SRDF)

### Cambiar el link tip de la cadena IK

Busca la línea 11 y actualiza `tip_link`:

```xml
<!-- Antes -->
<chain base_link="${prefix}arm_base_link" tip_link="${prefix}end_effector" />

<!-- Después -->
<chain base_link="${prefix}arm_base_link" tip_link="${prefix}suction_cup_tip" />
```

### Añadir reglas de desactivación de colisiones

Añade las siguientes entradas cerca de los bloques `disable_collisions` existentes. Estas evitan que MoveIt genere falsas alertas de colisión entre la ventosa y los links con los que siempre está en contacto.

```xml
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_finger_base_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_finger1_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_finger2_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_servo_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}arm_wrist_F3_0_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}arm_wrist_flex_link" reason="Never"/>
```

---

## Archivo 3: `phantomx_pincher.srdf` (SRDF desplegado)

Este archivo es equivalente al xacro pero usa nombres de links fijos (prefijo = `phantomx_pincher_`). Aplica los mismos dos cambios.

### Cambiar el link tip de la cadena IK

Busca la línea 9 y actualiza `tip_link`:

```xml
<!-- Antes -->
<chain base_link="phantomx_pincher_arm_base_link" tip_link="phantomx_pincher_end_effector"/>

<!-- Después -->
<chain base_link="phantomx_pincher_arm_base_link" tip_link="phantomx_pincher_suction_cup_tip"/>
```

### Añadir reglas de desactivación de colisiones

```xml
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_finger_base_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_finger1_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_finger2_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_servo_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_arm_wrist_F3_0_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_arm_wrist_flex_link" reason="Never"/>
```

---

## Cadena cinemática resultante

```
... → arm_wrist_flex_link
    → arm_wrist_F3_0_link
    → gripper_servo_link
    → gripper_finger_base_link
    → end_effector          (TCP anterior — se conserva como frame de referencia)
    → suction_cup_link      (malla con geometría visual y de colisión)
    → suction_cup_tip       (nuevo TCP usado por MoveIt para planificación IK)
```

---

## Tareas pendientes antes de finalizar

- [ ] Medir el desplazamiento físico desde el frame `end_effector` hasta la base del adaptador de ventosa y actualizar `xyz` en `end_effector_to_suction_cup_joint`.
- [ ] Medir la longitud de la ventosa y actualizar `joint_origin_xyz` en el joint del link virtual `suction_cup_tip` para situar el TCP exactamente en el punto de contacto.
- [ ] Verificar el origen de la malla en un visor 3D (Blender o MeshLab) para confirmar que los archivos `.dae` y `.stl` están alineados con el frame esperado antes de ejecutar.
