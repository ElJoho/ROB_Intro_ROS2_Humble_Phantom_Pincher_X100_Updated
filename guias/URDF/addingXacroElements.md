# Añadiendo elementos a archivos XACRO

Guía de procedimientos para añadir links y joints con distintas combinaciones de tags visuales y de colisión en el proyecto PhantomX Pincher ROS2 (archivos `kit.xacro` y `phantomx_pincher_arm.xacro`).

---

## Para elementos con un solo tag visual y un solo tag de colisiones

### Procedimiento general

1. Añadir archivos .DAE o .STL
2. Escribir estructura base de link y joint
3. Escribir el nombre del archivo en el tag de visual, así como añadir colores, etc.
4. Definir el largo, ancho y alto del ensamble/figura y asociarlos a los ejes x, y, z de RVIZ para hacer la figura de colisiones
5. Añadir valores de tag de colisión al archivo .xacro
6. Identificar las rotaciones necesarias para que el objeto visual quede en posición
7. Identificar la distancia del origen del ensamble/figura al origen de la base en Inventor
8. Comparar y comprobar que el objeto de colisión contenga al objeto visual

### Estructura base (link + joint)

```xml
<link name="name_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <mesh filename="package://phantomx_pincher_description/meshes/DAE/file.dae"
            scale="0.001 0.001 0.001" />
    </geometry>
    <material name="yellow" />
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.0 0.0 0.0" />
    </geometry>
  </collision>
</link>

<joint name="parent_child_joint" type="fixed">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
</joint>
```

### Ejemplo: acople de herramienta (suction cup) al brazo — `phantomx_pincher_arm.xacro`

1. Añadir archivos .DAE o .STL
2. Escribir estructura base de link y joint (ver plantilla abajo)
3. Escribir el nombre del archivo en el tag de visual, así como añadir colores, etc.
4. Identificar las rotaciones necesarias para que el objeto visual quede en posición
5. Identificar la distancia del origen del ensamble/figura al origen de la base en Inventor
6. Añadir valores al tag de colisión en `phantomx_pincher_arm.xacro` (usar malla STL si se requiere forma exacta, o figura primitiva con valores de alto, largo y ancho)
7. Identificar las rotaciones necesarias para que el objeto de colisión quede en posición
8. Comparar y comprobar que el objeto de colisión contenga al objeto visual

#### Plantilla específica (con `${prefix}` e `inertial`)

```xml
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
        <mass value="0.01"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.0001" ixy="0" ixz="0"
                 iyy="0.0001" iyz="0"
                 izz="0.0001"/>
    </inertial>
</link>

<joint name="${prefix}name_joint" type="fixed">
    <parent link="${prefix}arm_link"/>
    <child link="${prefix}name_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

---

## Para elementos con un solo tag visual y varios tags de colisiones

### Procedimiento general

1. Añadir archivos .DAE o .STL
2. Escribir estructura base de link y joint
3. Escribir el nombre del archivo en el tag de visual, así como añadir colores, etc.
4. Identificar las rotaciones necesarias para que el objeto visual quede en posición, y hacer las modificaciones al tag visual
5. Identificar la distancia del origen del ensamble/figura visual al origen de la base en Inventor, y hacer las modificaciones al tag joint
6. Definir qué geometrías se usarán para las colisiones y cuántas serán. Identificar información como su rotación también. Recordar que solo hay 3 geometrías primitivas (`cylinder`, `box`, `sphere`)
7. Definir el largo, ancho y alto de las distintas geometrías de colisión y asociarlos a los ejes x, y, z de RVIZ
8. En Inventor definir un origen para cada uno de los elementos de colisión que se trasladarán respecto al origen del joint
9. Añadir valores de tag de colisión al archivo .xacro

### Ejemplo: canastilla

1. Añadir archivos .DAE o .STL
   - Se añadió `canastilla.stl` a `meshes/STL/`

2. Escribir estructura base de link y joint
   - Se creó `canastilla_link` y `base_canastilla_joint` con parent `baseFija_link`

3. Escribir el nombre del archivo en el tag de visual, así como añadir colores, etc.
   - `mesh filename`: `package://phantomx_pincher_description/meshes/STL/canastilla.stl`
   - `scale`: `0.001 0.001 0.001`
   - `material`: `blue`
   - `origin xyz="0 0 0" rpy="0 0 0"`

4. Identificar las rotaciones necesarias para que el objeto visual quede en posición, y hacer las modificaciones al tag visual
   - No fue necesaria ninguna rotación adicional: `rpy="0 0 0"`

5. Identificar la distancia del origen del ensamble/figura visual al origen de la base en Inventor, y hacer las modificaciones al tag visual
   - `origin xyz="0.154 0 -${0.011 + 0.0145}" rpy="0 0 0"`
   - El origen del STL coincide con el origen del link, sin rotación adicional

6. Definir qué geometrías se usarán para las colisiones y cuántas serán. Identificar información como su rotación también. Recordar que solo hay 3 geometrías primitivas (`cylinder`, `box`, `sphere`)
   - 8 `box` en total:
     - 2 paredes largas (izquierda y derecha)
     - 2 paredes cortas (delantera y trasera)
     - 4 esquinas rotadas 45 grados (delantera izquierda, delantera derecha, trasera izquierda, trasera derecha)

7. Definir el largo, ancho y alto de las distintas geometrías de colisión y asociarlos a los ejes x, y, z de RVIZ
   - Paredes largas:   `box size="0.600 0.017 0.190"`
   - Paredes cortas:   `box size="0.017 0.366 0.190"`
   - Esquinas rotadas: `box size="0.018 0.018 0.190"`
   - El alto `0.190` es común a todas ya que la canastilla tiene altura uniforme

8. En Inventor definir un origen para cada uno de los elementos de colisión que se trasladarán respecto al origen del elemento visual
   - Pared larga izquierda:        `xyz="0  0.1915  0.095"`
   - Pared larga derecha:          `xyz="0 -0.1915  0.095"`
   - Pared corta delantera:        `xyz=" 0.2915  0.0    0.095"`
   - Pared corta trasera:          `xyz="-0.2915  0.0    0.095"`
   - Esquina trasera izquierda:    `xyz="-0.283   0.183  0.095"  rpy="0 0 PI/4"`
   - Esquina trasera derecha:      `xyz="-0.283  -0.183  0.095"  rpy="0 0 PI/4"`
   - Esquina delantera izquierda:  `xyz=" 0.283   0.183  0.095"  rpy="0 0 PI/4"`
   - Esquina delantera derecha:    `xyz=" 0.283  -0.183  0.095"  rpy="0 0 PI/4"`
   - El `z` de todas es `0.190/2 = 0.095` porque el origen está en la base de la canastilla

9. Añadir valores de tag de colisión a `kit.xacro`
   - Se añadieron los 8 tags de `collision` al `canastilla_link`

---

## Para elementos con varios tags visuales y varios tags de colisiones

### Procedimiento general

1. Añadir archivos .DAE o .STL
2. Escribir estructura base de link y joint
3. Identificar todos los tags visuales que se añadirán
4. Escribir los nombres de los archivos en los tags de visual, así como añadir colores, etc.
5. Identificar qué elementos del tag visual requieren rotaciones y qué rotaciones se van a hacer
6. Identificar los distintos orígenes de los tags visuales respecto del link padre. El origen del joint se deja en 0 (alineado con el origen del link padre) y luego se trasladan los elementos visuales
7. Definir el largo, ancho y alto de las distintas geometrías de colisión y asociarlos a los ejes x, y, z de RVIZ
8. En Inventor definir un origen para cada uno de los elementos de colisión que se trasladarán respecto al origen del joint
9. Añadir valores de tag de colisión al archivo .xacro

### Estructura base (link con varios visuales + una colisión, joint)

```xml
<link name="electronics_link">
  <visual> <!-- Multitoma -->
    <origin xyz="0.28225 -0.1515 ${0.0275 + 0.009}" rpy="0 0 ${PI/2}" />
    <geometry>
      <mesh filename="package://phantomx_pincher_description/meshes/DAE/ensambleMultitoma.dae"
            scale="0.001 0.001 0.001" />
    </geometry>
    <material name="yellow" />
  </visual>

  <visual> <!-- Arbotix -->
    <origin xyz="0.34796868605 0.11930139262 ${0.04542796419 + 0.009}" rpy="0 0 0" />
    <geometry>
      <mesh filename="package://phantomx_pincher_description/meshes/DAE/ensambleArbotix.dae"
            scale="0.001 0.001 0.001" />
    </geometry>
  </visual>

  <collision> <!-- bloque completo de electronica -->
    <origin xyz="${0.346 - 0.000125} -0.0375 ${(0.151 / 2) + 0.009}" rpy="0 0 0" />
    <geometry>
      <box size="${0.125 + 0.00325 + 0.0035} ${0.250 + 0.0245 + 0.0105} 0.151" />
    </geometry>
  </collision>
</link>

<joint name="base_electronics_joint" type="fixed">
  <parent link="baseFija_link" />
  <child link="electronics_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
</joint>
```

### Ejemplo: bloque de electrónica (Multitoma + Arbotix)

1. Añadir archivos .DAE o .STL
   - Se añadieron `ensambleMultitoma.dae` y `ensambleArbotix.dae` a `meshes/DAE/`

2. Escribir estructura base de link y joint
   - Se creó `electronics_link` y `base_electronics_joint` con parent `baseFija_link`

3. Identificar todos los tags visuales que se añadirán
   - 2 tags visuales:
     - Multitoma
     - Arbotix

4. Escribir los nombres de los archivos en los tags de visual, así como añadir colores, etc.
   - Multitoma:
     - `mesh filename`: `package://phantomx_pincher_description/meshes/DAE/ensambleMultitoma.dae`
     - `scale`: `0.001 0.001 0.001`
     - `material`: `yellow`
   - Arbotix:
     - `mesh filename`: `package://phantomx_pincher_description/meshes/DAE/ensambleArbotix.dae`
     - `scale`: `0.001 0.001 0.001`
     - Sin material definido (usa el por defecto)

5. Identificar qué elementos del tag visual requieren rotaciones y qué rotaciones se van a hacer
   - Multitoma: `rpy="0 0 PI/2"` (rotación de 90 grados en z)
   - Arbotix:   `rpy="0 0 0"` (sin rotación adicional)

6. Identificar los distintos orígenes de los tags visuales respecto del link padre. El origen del joint se deja en 0 (alineado con el origen del link padre) y luego se trasladan los elementos visuales
   - `joint origin`: `xyz="0 0 0" rpy="0 0 0"`
   - Multitoma: `xyz="0.28225 -0.1515 0.0365"`
     - `z = 0.0275 + 0.009` (altura del elemento + grosor de la base)
   - Arbotix:   `xyz="0.34796868605 0.11930139262 0.05442796419"`
     - `z = 0.04542796419 + 0.009` (altura del elemento + grosor de la base)

7. Definir el largo, ancho y alto de las distintas geometrías de colisión y asociarlos a los ejes x, y, z de RVIZ
   - 1 `box` que engloba todo el bloque de electrónica:
     - `x = 0.125 + 0.00325 + 0.0035`  (ancho total del bloque)
     - `y = 0.250 + 0.0245 + 0.0105`   (largo total del bloque)
     - `z = 0.151`                     (alto total del bloque)

8. En Inventor definir un origen para cada uno de los elementos de colisión que se trasladarán respecto al origen del joint
   - Bloque completo de electrónica: `xyz="0.345875 -0.0375 0.0845"`
     - `x = 0.346 - 0.000125` (centro del bloque en x)
     - `y = -0.0375`           (centro del bloque en y)
     - `z = 0.151/2 + 0.009`   (mitad del alto + grosor de la base)

9. Añadir valores de tag de colisión a `kit.xacro`
   - Se añadió 1 tag de `collision` al `electronics_link` cubriendo ambos elementos visuales con un solo `box`