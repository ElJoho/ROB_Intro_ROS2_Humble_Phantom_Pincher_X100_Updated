# Añadiendo elementos a archivos XACRO


**Autor:** Johan Alejandro López Arias ([@ElJoho](https://github.com/ElJoho))


Guía de procedimientos para añadir links y joints con distintas combinaciones de tags visuales y de colisión en el proyecto PhantomX Pincher ROS2 (archivos `kit.xacro` y `phantomx_pincher_arm.xacro`).

<summary>📺 Serie de videos en YouTube (10 episodios)</summary>

<details>
<summary><b>Ep. 1</b> — <a href="https://www.youtube.com/watch?v=12hgfwh_EL4">Introducción a URDF</a></summary>

Video introductorio a URDF y xacro en el contexto del kit Phantom X Pincher, explicando la estructura del archivo `kit.xacro` (links, joints, tags `<visual>` y `<collision>`) y la diferencia entre los archivos DAE (malla con información visual) y STL (malla geométrica pura), incluyendo el detalle de la escala `0.001` para convertir de milímetros de Inventor a los metros que espera RViz. Se muestra la distinción clave entre el "mundo visual" (bonito pero ignorado por el planificador) y el "mundo de las colisiones" (el que realmente importa para el movimiento del robot), usando RViz con el launch del paquete `urdf_tutorial` y ajustando el archivo de configuración `fixed.rviz` (cambiar `base_link` por `world_link`, alfa a 1, escala del marcador a 0.1). Finaliza mostrando cómo usar rotaciones del `world_link` para simular vistas ortogonales en RViz y corrigiendo en vivo un problema de dimensionamiento de la cámara y su acople, dejando el terreno preparado para los siguientes videos donde se añadirán nuevos elementos al kit.

</details>

<details>
<summary><b>Ep. 2</b> — <a href="https://www.youtube.com/watch?v=lNkqghHquOs">Un elemento con 1 tag visual y 1 tag de colisión</a></summary>

Video que muestra el flujo completo para añadir a `kit.xacro` un link con un único tag `<visual>` y un único `<collision>`, usando como ejemplo el ensamble de la Raspberry Pi sobre la base. Se recorren los pasos clave: copiar la estructura base cambiando siempre el nombre del link y del joint para evitar errores, referenciar el archivo DAE con su material, asociar largo/ancho/alto de la figura a los ejes X/Y/Z de RViz (que difieren de la orientación de Inventor), y obtener las traslaciones del origen creando un punto por intersección de tres planos en Inventor y sumando los 9 mm de grosor de la base en Z. Se explica además que las rotaciones conviene hacerlas en el tag `<visual>` (y no en el joint) para que este último no arrastre el tag de colisión, cerrando con la verificación en RViz de que la caja de colisión contiene correctamente al objeto visual y aclarando que el orden de los pasos es una guía flexible, no obligatoria.

</details>

<details>
<summary><b>Ep. 3</b> — <a href="https://www.youtube.com/watch?v=uBKfdnKrj4s">Un elemento con 1 tag visual y varios tags de colisión</a></summary>

Video que muestra cómo definir en `kit.xacro` un link con un único tag `<visual>` pero varios tags `<collision>`, usando como ejemplo la canastilla: se importa el archivo STL (más liviano que un DAE al ser una pieza de un solo color) asignando el material directamente desde xacro, y se calculan las traslaciones del origen del joint midiendo distancias en Inventor mediante puntos de intersección de tres planos. Las colisiones se aproximan con ocho cajas primitivas (dos paredes largas, dos cortas y cuatro prismas rotados 45° en Z para cubrir las esquinas redondeadas), calculando cuidadosamente sus dimensiones y desplazamientos teniendo en cuenta que el origen de cada caja queda en su centro geométrico, por lo que la altura se divide entre dos. Se compara brevemente contra la alternativa de usar el STL completo como colisión mediante el tag `<mesh>`, descartándola por el sobrecosto computacional de geometría innecesaria, y se enfatiza la importancia de nombrar cada tag de colisión con un comentario para facilitar futuras ediciones.

</details>

<details>
<summary><b>Ep. 4</b> — <a href="https://www.youtube.com/watch?v=ETsq8yryuyo">1 elemento con varios tags visuales y varios tags de colisión</a></summary>

Video que muestra cómo definir un mismo link con varios tags `<visual>` y uno o más tags `<collision>` en `kit.xacro`, tomando como ejemplo el link `electronic_link` que contiene los ensambles DAE de la Arbotix y la multitoma dentro de un único bloque de colisión tipo caja. Se explica por qué conviene dejar el origen del joint en cero (alineado con la base) y trasladar cada elemento visual respecto al origen del link padre, midiendo en Inventor los desplazamientos de cada pieza con puntos de intersección de tres planos y sumando el grosor de 9 mm de la base. Para el bloque de colisión se calculan sus dimensiones (largo, ancho y altura de 151 mm) a partir del layout de la plataforma, verificando finalmente en RViz que todo queda alineado con los tornillos y contenido dentro de la caja de colisiones, y cerrando con instrucciones para el laboratorio sobre cómo personalizar la canastilla con el número del robot y los nombres del equipo.

</details>

<details>
<summary><b>Ep. 5</b> — <a href="https://www.youtube.com/watch?v=gWCVxNBr8pw">Desactivando y activando colisiones de kit.xacro</a></summary>

Introducción al manejo de colisiones en MoveIt editando los archivos del paquete `phantomx_pincher_moveit_config` (carpeta `srdf`), eliminando primero las referencias a los links obsoletos `manija_trasera_link` y `manija_delantera_link` que ahora forman parte de la base fija. Se explica la estructura del tag `disable_collisions` con sus razones convencionales (`never`, `adjacent`, `default`, `always`, `user`) y se añaden las desactivaciones necesarias para los nuevos links de Raspberry Pi, electrónica y soporte de cámara para evitar falsos positivos por contactos marginales. Cierra compilando con `colcon` desde la raíz del workspace y verificando en MoveIt que el modelo carga correctamente, dejando visible el acople del efector final que se abordará en los siguientes videos.

</details>

<details>
<summary><b>Ep. 6</b> — <a href="https://www.youtube.com/watch?v=ELiEouOz8Vs">Convertir archivos IPT de Inventor a DAE con Blender 4.2 LTS</a></summary>

Video corto que muestra el flujo para convertir piezas de Inventor a archivos DAE (colada) usando Blender 4.2 LTS como paso intermedio, exportando primero desde Inventor al formato OBJ (junto con su MTL). Se aclara que debe usarse específicamente Blender 4.2 LTS o versiones anteriores porque el soporte de exportación a DAE fue descontinuado en versiones posteriores, y que al importar el OBJ es necesario seleccionar también el archivo MTL para conservar los materiales. El proceso se repite para dos piezas (el gripper y el acople), dejando listos los archivos DAE que se usarán en el siguiente video para integrar el acople al brazo robótico.

</details>

<details>
<summary><b>Ep. 7</b> — <a href="https://www.youtube.com/watch?v=UJGw7AIaVZ0">Agregar elementos al brazo con phantomx_pincher_arm.xacro</a></summary>

Video que explica cómo añadir un nuevo elemento (el acople de la ventosa) editando directamente `phantomx_pincher_arm.xacro`, resaltando las diferencias frente a `kit.xacro`: uso de macros, manejo obligatorio de prefijos en nombres y referencias padre/hijo, y reutilización de los colores ya definidos sin redeclararlos. Se muestra cómo obtener los momentos de inercia y centro de masa desde Inventor (convirtiendo de mm² a m²), identificar rotaciones usando los frames en RViz y calcular la traslación del joint restando las posiciones publicadas en `/tf` entre el servo padre y el acople. Incluye la solución a errores típicos de tags mal cerrados junto al flujo de `colcon build`, `source` y ejecución del launch, cerrando con la verificación visual de que la colisión contiene al objeto visual.

</details>

<details>
<summary><b>Ep. 8</b> — <a href="https://www.youtube.com/watch?v=71B-XrqGfXc">Desactivar colisiones de elementos nuevos en brazo robótico</a></summary>

Segunda parte del proceso de añadir un acople al brazo robótico, enfocada en desactivar las colisiones entre el nuevo link y los elementos cercanos del efector final editando los archivos xacro y SRDF del paquete `phantom_x_pincher_moveit_config`. Se muestra cómo identificar qué links pueden entrar en contacto usando RViz (activando y desactivando su visualización) y cómo aplicar el tag `disable_collisions` con el prefijo correcto, incluyendo un reemplazo manual del prefijo en el SRDF debido a un bug de compilación no resuelto. Finalmente se verifica en MoveIt que el planificador genera trayectorias sin reportar colisiones, dejando lista la base para añadir la ventosa en el siguiente video.

</details>

<details>
<summary><b>Ep. 9</b> — <a href="https://www.youtube.com/watch?v=6Z_Q9niSPAY">Agregar suction cup a phantomx_pincher_arm.xacro</a></summary>

Video largo y sin guion donde se añade la ventosa (suction cup) al archivo `phantomx_pincher_arm.xacro`, replicando el proceso del acople anterior pero usándolo ahora como link padre del nuevo joint. Se muestran en vivo los errores típicos y cómo corregirlos: malas rotaciones del joint interpretando el marco de referencia equivocado, cálculo de las traslaciones tomando medidas desde Inventor, y ajuste del objeto de colisión (rotación y traslación) cuando el origen del STL exportado no coincide con el del archivo DAE. Cierra desactivando las colisiones entre la ventosa y la garra editando tanto el xacro como el SRDF de `phantomx_pincher_moveit_config`, y verificando en MoveIt que el modelo completo planea trayectorias sin problemas.

</details>

<details>
<summary><b>Ep. 10</b> — <a href="https://www.youtube.com/watch?v=GsfinJpJwyY">Cambiar posición del TCP</a></summary>

Video de cierre de la serie de URDF donde se explica cómo cambiar el origen del TCP en el archivo xacro del Phantom X Pincher, manejando dos TCPs (con y sin ventosa) mediante un condicional controlado por la variable `use_suction`. Se muestra cómo obtener las coordenadas del nuevo TCP desde Inventor y ajustar la rotación en Y para conservar la orientación del marco del efector final, además de corregir un error previo sobre el link padre del acople de la ventosa. Incluye una prueba práctica moviendo el robot con los comandos de MoveIt para verificar que el TCP se desplaza a la posición correcta.

</details>

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