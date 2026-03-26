## Añadiendo acople de herramienta . suction cup a brazo. phantomx_pincher_arm.xacro

**1. Añadir archivos .DAE o .STL**
- `acopleSuctionCupGripper.dae` → carpeta `meshes/DAE/`
- `acopleSuctionCupGripper.stl` → carpeta `meshes/STL/`

---

**2. Escribir estructura base de link y joint**
- Nombres usados:
  - Link: `${prefix}coupling_tool_end_effector_link`
  - Joint: `${prefix}acople_suction_cup_joint`
  - Parent link del joint: `${prefix}end_effector`

```xacro
    <link name="${prefix}name_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="package://phantomx_pincher_description/meshes/DAE/stlFile.dae"
                scale="0.001 0.001 0.001" />
            </geometry>
            <material name="gray" />
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="package://phantomx_pincher_description/meshes/STL/stlFile.stl"
                scale="0.001 0.001 0.001" />
            </geometry>
        </collision>
        <inertial>
            < inertia
            ixx="1.5083e-05"  ixy="0.0"       ixz="5.9e-08"
                              iyy="2.123e-06" iyz="0.0"
                                              izz="1.6244e-05"/>
        </inertial>
    </link>
  
    <joint name="${prefix}name_joint" type="fixed">
        <parent link="${prefix}arm_link"/>
        <child link="${prefix}name_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
```

---


**3. Escribir el nombre del archivo en el tag de visual, así como añadir colores, etc**
- Mesh visual: `acopleSuctionCupGripper.dae`
- Scale: `0.001 0.001 0.001` (conversión mm → m)
- Material: `gray`

---

**4. Identificar las rotaciones necesarias para que el objeto visual quede en posición**
- La rotación del visual se dejó en `rpy="0 0 0"` (sin rotación local)
- La rotación necesaria se aplicó en el **joint**: `rpy="${PI/2} 0 ${PI/2}"`

---

**5. Identificar la distancia del origen del ensamble/figura al origen de la base en Inventor**
- Distancia del joint (origen del acople respecto al parent `end_effector`):
  - `xyz="0.000008282 0 -0.038"` → desplazamiento de **-38 mm en Z**, con offset mínimo en X (`~0.008 mm`, probablemente residuo del ensamble en Inventor)

---

**6. Añadir valores de tag de colisión — malla STL**
- Se usó **malla STL** (no figura primitiva):
  - Mesh colisión: `acopleSuctionCupGripper.stl`
  - Scale: `0.001 0.001 0.001`

---

**7. Identificar las rotaciones necesarias para que el objeto colisión quede en posición**
- Origin de colisión: `xyz="0 -${0.0215/2} 0.0173"` , `rpy="-${PI/2} 0 0"`
  - Rotación de **-90°** en X para alinear la malla STL con la orientación del visual
  - Offset en Y de `-0.01075 m` (mitad de 21.5 mm) y `+0.0173 m` en Z para centrar la geometría de colisión sobre la visual

---

**8. Comparar y comprobar que el objeto de colisión contenga al objeto visual**
- Se verificó que con la rotación `-${PI/2}` en X y el offset `xyz="0 -${0.0215/2} 0.0173"` la malla de colisión envuelve correctamente a la malla visual del acople

---

**9. Desactivar las colisiones entre los links del brazo que estan en contacto y los links añadidos.**

```xacro
      <!-- ============================== -->
      <!-- B) COUPLING SELF COLLISIONS OFF   -->
      <!-- ============================== -->
      <!-- coupling_tool_end_effector_link collisions -->
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}end_effector" reason="Adjacent" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}gripper_finger_base_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}gripper_finger1_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}gripper_finger2_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}gripper_servo_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_wrist_flex_servo_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_wrist_flex_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_wrist_F3_0_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_elbow_flex_servo_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_elbow_flex_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_elbow_F3_0_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_elbow_F10_0_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_elbow_F10_1_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_elbow_F10_2_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_shoulder_F10_0_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_shoulder_F10_1_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_shoulder_F10_2_link" reason="Never" />
      <disable_collisions link1="${prefix}coupling_tool_end_effector_link"
        link2="${prefix}arm_shoulder_F3_0_link" reason="Never" />
```