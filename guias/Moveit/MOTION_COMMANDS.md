# PhantomX Pincher — Referencia de Comandos de Movimiento

> Generado a partir del análisis del workspace de los paquetes:
> `phantomx_pincher`, `phantomx_pincher_bringup`, `pincher_control`,
> `phantomx_pincher_moveit_config`, `phantomx_pincher_commander_cpp`

---

## Tabla de Contenidos

1. [Visión General de la Arquitectura](#1-visión-general-de-la-arquitectura)
2. [Comandos de Lanzamiento Requeridos](#2-comandos-de-lanzamiento-requeridos)
3. [Modalidad 1 — Control Directo de Motores (ticks Dynamixel)](#3-modalidad-1--control-directo-de-motores-ticks-dynamixel)
4. [Modalidad 2 — Meta Cartesiana con MoveIt](#4-modalidad-2--meta-cartesiana-con-moveit)
5. [Modalidad 3 — Control del Gripper por Ticks](#5-modalidad-3--control-del-gripper-por-ticks)
6. [Modalidad 4 — Control del Gripper por Apertura (MoveIt)](#6-modalidad-4--control-del-gripper-por-apertura-moveit)
7. [Modalidad 5 — Estados Nombrados de MoveIt (Move Groups en RViz)](#7-modalidad-5--estados-nombrados-de-moveit-move-groups-en-rviz)
8. [Todos los Tópicos, Servicios y Acciones](#8-todos-los-tópicos-servicios-y-acciones)
9. [Referencia de Nodos](#9-referencia-de-nodos)
10. [Tarjeta de Referencia Rápida](#10-tarjeta-de-referencia-rápida)

---

## 1. Visión General de la Arquitectura

```
┌───────────────────────────────────────────────────────────────────────┐
│                        COMANDOS DEL USUARIO                           │
│  ros2 topic pub  /  ros2 run  /  ros2 action  /  RViz Motion Planning │
└──────────────┬────────────────────────────┬───────────────────────────┘
               │                            │
               ▼                            ▼
  ┌────────────────────────┐   ┌──────────────────────────┐
  │  nodo commander         │   │  move_group (MoveIt)     │
  │  (commander_template)   │   │  - planificador OMPL     │
  │  Suscripciones:         │   │  - ejecución trayectoria │
  │  /open_gripper          │   │  - estados nombrados     │
  │  /joint_command         │   └────────────┬─────────────┘
  │  /pose_command          │                │ goals FollowJointTrajectory
  └────────────┬────────────┘                │
               │ MoveGroupInterface          │
               └──────────────┬─────────────┘
                              │
              ┌───────────────┴───────────────┐
              │                               │
              ▼ (SIMULACIÓN)                  ▼ (HARDWARE REAL)
  ┌────────────────────────┐    ┌──────────────────────────────────┐
  │ ros2_control / fake hw  │    │  pincher_follow_joint_trajectory  │
  │ joint_trajectory_ctrl   │    │  (follow_joint_trajectory_node)  │
  │ gripper_trajectory_ctrl │    │  AX-12A Dynamixel via USB/U2D2   │
  └────────────────────────┘    └──────────────────────────────────┘
```

### Mapeo Servo ID → Joint (Hardware Real)

| Nombre del Joint                                 | ID Dynamixel |
|--------------------------------------------------|:------------:|
| `phantomx_pincher_arm_shoulder_pan_joint`        | 1            |
| `phantomx_pincher_arm_shoulder_lift_joint`       | 2            |
| `phantomx_pincher_arm_elbow_flex_joint`          | 3            |
| `phantomx_pincher_arm_wrist_flex_joint`          | 4            |
| `phantomx_pincher_gripper_finger1_joint`         | 5 (gripper)  |
| `phantomx_pincher_gripper_finger2_joint`         | 5 (gripper)  |

Ambos joints del dedo comparten el mismo servo físico (ID 5).

---

## 2. Comandos de Lanzamiento Requeridos

### 2.1 Modo Simulación (Hardware Falso + RViz + MoveIt)

Lanza MoveIt con hardware falso de ros2_control y RViz:

```bash
ros2 launch phantomx_pincher fake.launch.py
```

O equivalentemente:

```bash
ros2 launch phantomx_pincher_moveit_config move_group.launch.py \
  ros2_control:=true \
  ros2_control_plugin:=fake
```

**Inicia:** `robot_state_publisher`, `ros2_control_node` (hw falso), `move_group`, `rviz2`,
`joint_trajectory_controller`, `gripper_trajectory_controller`.

---

### 2.2 Modo Robot Real (Stack Completo)

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true
```

**Inicia:**
- `robot_state_publisher`
- Nodo `commander` (wrapper MoveGroupInterface)
- Nodo `pincher_follow_joint_trajectory` (driver de hardware Dynamixel)
- `move_group` (MoveIt, conectado a los action servers del hardware real)
- `rviz2`

**Puerto por defecto:** `/dev/ttyUSB0` — se puede cambiar con el parámetro `port`.

---

### 2.3 Modo Simulación via Bringup (con Commander)

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py
# use_real_robot toma el valor "false" por defecto
```

Lanza el stack completo de simulación incluyendo el nodo `commander`.

---

### 2.4 Iniciar Solo el Driver de Hardware (sin MoveIt)

```bash
ros2 run pincher_control follow_joint_trajectory \
  --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000
```

Expone dos action servers para ejecución directa de trayectorias sin la sobrecarga de MoveIt.

---

## 3. Modalidad 1 — Control Directo de Motores (ticks Dynamixel)

Estos comandos evitan MoveIt por completo y envían valores de ticks crudos (0–1023)
directamente a los servos AX-12A a través del bus Dynamixel.

**Referencia de ticks AX-12A:**
- Rango: 0–1023
- Centro (0 rad): ~512 ticks
- Rango completo: 0°–300° mapeado a 0–1023 ticks

### 3.1 `control_servo` — Comando de Ticks de Una Sola Vez (Sin GUI)

**Nodo:** `pincher_control/control_servo`
**Propósito:** Enviar un único conjunto de posiciones objetivo en ticks a cualquier
subconjunto de servos, esperar la finalización y apagarse.

**Sintaxis:**
```bash
ros2 run pincher_control control_servo --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=1000000 \
  -p dxl_ids:=[1,2,3,4,5] \
  -p goal_positions:=[<t1>,<t2>,<t3>,<t4>,<t5>] \
  -p moving_speed:=<velocidad> \
  -p torque_limit:=<limite> \
  -p delay:=<segundos>
```

**Parámetros:**

| Parámetro        | Tipo       | Por defecto             | Descripción                                    |
|------------------|------------|-------------------------|------------------------------------------------|
| `port`           | string     | `/dev/ttyUSB0`          | Puerto serie del adaptador U2D2               |
| `baudrate`       | int        | `1000000`               | Baudrate del bus Dynamixel                    |
| `dxl_ids`        | int[]      | `[1,2,3,4,5]`           | Lista de IDs de servo a comandar              |
| `goal_positions` | int[]      | `[512,512,512,512,512]` | Valores de ticks objetivo (misma longitud que IDs) |
| `moving_speed`   | int        | `100`                   | Velocidad del motor 0–1023 (0=velocidad máxima) |
| `torque_limit`   | int        | `1000`                  | Límite de torque 0–1023                       |
| `delay`          | float      | `2.0`                   | Segundos a esperar para completar el movimiento |

**Ejemplo — Mover pan al centro, lift abajo, resto del brazo neutro:**
```bash
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[1,2,3,4] \
  -p goal_positions:=[512,200,700,512] \
  -p moving_speed:=150 \
  -p torque_limit:=800 \
  -p delay:=3.0
```

**Comportamiento esperado:** Los motores se mueven a las posiciones en ticks especificadas
a la velocidad dada. Tras `delay` segundos, el torque se desactiva y el nodo termina.

---

### 3.2 `control_servo` — GUI Interactiva con Sliders

**Propósito:** Abrir una GUI de Tkinter con sliders por motor (tiempo real) y un panel
de entrada de valores manuales. También integra visualización en RViz.

```bash
ros2 run pincher_control control_servo
```

**Pestañas de la GUI:**
- **Control por Sliders** — Sliders en tiempo real (rango 0–4095) para cada uno de los
  5 motores, más un slider de velocidad.
- **Control por Valores** — Introducir un valor de ticks específico por motor y pulsar
  "Mover Motor". Pulsar "MOVER TODOS LOS MOTORES" para comandar todos a la vez.
- **Visualización RViz** — Botón para lanzar `phantomx_pincher_description/display.launch.py`
  y muestra las posiciones actuales de los joints en radianes.

**Botones comunes (todas las pestañas):**
- **HOME** — Enviar todos los motores a 512 ticks (posición central, 0 rad).
- **PARADA DE EMERGENCIA** — Deshabilitar el torque en todos los motores inmediatamente.

**Comportamiento esperado:** Mover un slider envía inmediatamente el valor de ticks al
servo correspondiente. Los joint states se publican a 10 Hz en `/joint_states`
para la visualización en RViz.

---

### 3.3 Action Server — FollowJointTrajectory (trayectoria directa)

El nodo `follow_joint_trajectory` expone dos action servers que aceptan goals de tipo
`control_msgs/action/FollowJointTrajectory`. Se pueden llamar directamente desde la
terminal usando `ros2 action`.

**Prerrequisito:** El driver de hardware debe estar ejecutándose:
```bash
ros2 run pincher_control follow_joint_trajectory
```

**Action server del brazo:**
```
/joint_trajectory_controller/follow_joint_trajectory
```

**Action server del gripper:**
```
/gripper_trajectory_controller/follow_joint_trajectory
```

**Ejemplo — Mover el brazo a una posición articular con un solo punto:**
```bash
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [
        phantomx_pincher_arm_shoulder_pan_joint,
        phantomx_pincher_arm_shoulder_lift_joint,
        phantomx_pincher_arm_elbow_flex_joint,
        phantomx_pincher_arm_wrist_flex_joint
      ],
      points: [{
        positions: [0.0, 0.0, 1.5707963, 1.5707963],
        time_from_start: {sec: 3, nanosec: 0}
      }]
    }
  }"
```

**Los valores están en radianes** (convertidos internamente a ticks mediante: `tick = 512 + deg × 1023/300`).

**Ejemplo — Múltiples puntos de paso:**
```bash
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [
        phantomx_pincher_arm_shoulder_pan_joint,
        phantomx_pincher_arm_shoulder_lift_joint,
        phantomx_pincher_arm_elbow_flex_joint,
        phantomx_pincher_arm_wrist_flex_joint
      ],
      points: [
        {positions: [0.0, 0.0, 0.0, 0.0],   time_from_start: {sec: 2, nanosec: 0}},
        {positions: [0.5, 0.3, 1.0, 1.0],   time_from_start: {sec: 5, nanosec: 0}},
        {positions: [0.0, 0.0, 0.0, 0.0],   time_from_start: {sec: 8, nanosec: 0}}
      ]
    }
  }"
```

**Comportamiento esperado:** El nodo recorre los puntos de la trayectoria en orden,
durmiendo hasta cada `time_from_start` y luego comandando las posiciones de los servos.
Los joint states se publican a 50 Hz con las lecturas reales del hardware.

---

## 4. Modalidad 2 — Meta Cartesiana con MoveIt

MoveIt calcula una solución IK y planifica una trayectoria articular para alcanzar
una pose objetivo del efector final. El movimiento se ejecuta a través del nodo
`commander` o del panel de Motion Planning de RViz.

**Prerrequisito:** El stack completo de MoveIt debe estar ejecutándose (cualquier
comando de lanzamiento del §2).

### 4.1 Vía el Tópico `pose_command` (nodo commander)

**Tópico:** `/pose_command`
**Tipo de mensaje:** `phantomx_pincher_interfaces/msg/PoseCommand`

**Campos del mensaje:**
```
float64 x            # Posición X en metros (frame de planificación)
float64 y            # Posición Y en metros
float64 z            # Posición Z en metros
float64 roll         # Orientación roll en radianes
float64 pitch        # Orientación pitch en radianes
float64 yaw          # Orientación yaw en radianes
bool    cartesian_path  # true = trayectoria Cartesiana en línea recta; false = planificación libre en espacio articular
```

**Sintaxis:**
```bash
ros2 topic pub --once /pose_command \
  phantomx_pincher_interfaces/msg/PoseCommand \
  "{x: <X>, y: <Y>, z: <Z>, roll: <R>, pitch: <P>, yaw: <W>, cartesian_path: <true|false>}"
```

**Ejemplo — Mover el efector final a la posición (0.15, 0.0, 0.10) apuntando hacia abajo:**
```bash
ros2 topic pub --once /pose_command \
  phantomx_pincher_interfaces/msg/PoseCommand \
  "{x: 0.15, y: 0.0, z: 0.10, roll: 3.14159, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```

**Ejemplo — Trayectoria Cartesiana en línea recta hasta la misma pose:**
```bash
ros2 topic pub --once /pose_command \
  phantomx_pincher_interfaces/msg/PoseCommand \
  "{x: 0.15, y: 0.0, z: 0.10, roll: 3.14159, pitch: 0.0, yaw: 0.0, cartesian_path: true}"
```

**Comportamiento esperado (cartesian_path: false):** MoveIt usa OMPL para encontrar
una trayectoria articular libre de colisiones desde la pose actual hasta la objetivo.
La velocidad y aceleración se escalan al 100% por defecto (configurado en `commander_template.cpp`).

**Comportamiento esperado (cartesian_path: true):** MoveIt usa `computeCartesianPath`
con resolución de 1 cm. La trayectoria solo se ejecuta si el 100% del camino es
alcanzable. Útil para movimientos de aproximación precisos en pick-and-place.

---

### 4.2 Vía el Panel de Motion Planning de RViz (Marcador Interactivo)

1. Abrir RViz (iniciado por cualquier comando de lanzamiento del §2).
2. En el panel **Motion Planning**, usar el marcador interactivo (esfera/flecha naranja)
   para arrastrar el efector final a la pose Cartesiana deseada.
3. Hacer clic en **Plan** para calcular la trayectoria.
4. Hacer clic en **Execute** para ejecutarla en el robot.
5. O hacer clic en **Plan & Execute** para hacer ambos en un solo paso.

**Nota:** El frame de planificación reportado por el `MoveGroupInterface` del grupo `arm`
se usa para todos los objetivos de pose. El nodo `commander` registra el frame de
planificación al iniciarse.

---

## 5. Modalidad 3 — Control del Gripper por Ticks

### 5.1 Comando directo de ticks al servo del gripper (control_servo)

El servo del gripper tiene ID 5. Usar `control_servo` para enviar un valor de ticks crudo:

```bash
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[5] \
  -p goal_positions:=[<tick>] \
  -p moving_speed:=100 \
  -p delay:=2.0
```

**Calibración de ticks del gripper:**

| Estado  | Ticks | Apertura física (total) |
|---------|:-----:|:-----------------------:|
| Cerrado | 0     | 7.5 mm                  |
| Abierto | 520   | 39.1 mm                 |

**Ejemplo — Abrir gripper completamente:**
```bash
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[5] \
  -p goal_positions:=[520] \
  -p moving_speed:=100 \
  -p delay:=2.0
```

**Ejemplo — Cerrar gripper completamente:**
```bash
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[5] \
  -p goal_positions:=[0] \
  -p moving_speed:=100 \
  -p delay:=2.0
```

**Ejemplo — Gripper a media apertura (≈260 ticks):**
```bash
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[5] \
  -p goal_positions:=[260] \
  -p moving_speed:=80 \
  -p delay:=2.0
```

### 5.2 Vía el Action Server del Gripper (conversión ticks → metros interna)

El nodo `follow_joint_trajectory` acepta posiciones del gripper en **metros por dedo**
y las convierte internamente a ticks. La fórmula de conversión es:

```
t = (metros - 0.00375) / (0.01955 - 0.00375)
tick = round(t × 520)    # limitado al rango [0, 1023]
```

**Ejemplo — Abrir gripper vía action (0.01955 m por dedo = abierto):**
```bash
ros2 action send_goal \
  /gripper_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [
        phantomx_pincher_gripper_finger1_joint,
        phantomx_pincher_gripper_finger2_joint
      ],
      points: [{
        positions: [0.01955, 0.01955],
        time_from_start: {sec: 2, nanosec: 0}
      }]
    }
  }"
```

**Ejemplo — Cerrar gripper vía action (0.00375 m por dedo = cerrado):**
```bash
ros2 action send_goal \
  /gripper_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [
        phantomx_pincher_gripper_finger1_joint,
        phantomx_pincher_gripper_finger2_joint
      ],
      points: [{
        positions: [0.00375, 0.00375],
        time_from_start: {sec: 2, nanosec: 0}
      }]
    }
  }"
```

---

## 6. Modalidad 4 — Control del Gripper por Apertura (MoveIt)

Estos comandos usan el grupo de planificación del gripper de MoveIt y envían valores
de apertura en **metros por dedo** (no apertura total).

**Referencia física:**

| Estado  | Valor por dedo (m) | Apertura total (mm) |
|---------|:------------------:|:-------------------:|
| Abierto | 0.01955            | 39.1                |
| Cerrado | 0.00375            | 7.5                 |

### 6.1 Vía el Tópico `open_gripper` (nodo commander)

**Tópico:** `/open_gripper`
**Tipo de mensaje:** `example_interfaces/msg/Bool`

El nodo `commander` establece un estado nombrado de MoveIt:
- `true` → `setNamedTarget("open")` → abre el gripper
- `false` → `setNamedTarget("closed")` → cierra el gripper

```bash
# Abrir gripper
ros2 topic pub --once /open_gripper \
  example_interfaces/msg/Bool "{data: true}"

# Cerrar gripper
ros2 topic pub --once /open_gripper \
  example_interfaces/msg/Bool "{data: false}"
```

### 6.2 Vía el Action de Trayectoria del Gripper con Metros (pipeline MoveIt)

Tras lanzar MoveIt, publicar al action del grupo gripper directamente con
valores de apertura en metros. MoveIt planifica la trayectoria:

```bash
# Abierto (0.01955 m por dedo)
ros2 action send_goal \
  /gripper_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
      joint_names: [phantomx_pincher_gripper_finger1_joint, phantomx_pincher_gripper_finger2_joint],
      points: [{positions: [0.01955, 0.01955], time_from_start: {sec: 2, nanosec: 0}}]
  }}"

# Cerrado (0.00375 m por dedo)
ros2 action send_goal \
  /gripper_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
      joint_names: [phantomx_pincher_gripper_finger1_joint, phantomx_pincher_gripper_finger2_joint],
      points: [{positions: [0.00375, 0.00375], time_from_start: {sec: 2, nanosec: 0}}]
  }}"
```

### 6.3 Vía el Panel de Motion Planning de RViz — Grupo Gripper

1. En el panel **Motion Planning** de RViz, establecer el desplegable **Planning Group** a `gripper`.
2. Establecer **Goal State** a `open` o `closed`.
3. Hacer clic en **Plan & Execute**.

---

## 7. Modalidad 5 — Estados Nombrados de MoveIt (Move Groups en RViz)

Los estados nombrados son configuraciones articulares predefinidas almacenadas en el
archivo SRDF en:
`phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf`

### 7.1 Referencia de Estados Nombrados

#### Grupo: `arm`

| Nombre del Estado  | shoulder_pan | shoulder_lift | elbow_flex | wrist_flex |
|--------------------|:------------:|:-------------:|:----------:|:----------:|
| `up`               | 0.0          | 0.0           | 0.0        | 0.0        |
| `rest`             | 0.0          | -1.5708       | 2.2689     | 1.5708     |
| `ready_near`       | 0.0          | 0.0           | 1.5708     | 1.5708     |
| `ready_mid`        | 0.0          | 0.3491        | 1.5708     | 1.2217     |
| `ready_far`        | 0.0          | 1.5708        | 0.0        | 1.5708     |
| `overRightNearCan` | -1.5708      | 0.3665        | 1.3963     | 1.3963     |

> Todos los valores en radianes.

#### Grupo: `gripper`

| Nombre del Estado | finger1_joint (m) | finger2_joint (m) | Apertura total |
|-------------------|:-----------------:|:-----------------:|:--------------:|
| `open`            | 0.01955           | 0.01955           | 39.1 mm        |
| `closed`          | 0.00375           | 0.00375           | 7.5 mm         |

---

### 7.2 Usar Estados Nombrados vía el Panel de Motion Planning de RViz

**Prerrequisito:** Stack de MoveIt ejecutándose (cualquier lanzamiento del §2).

**Flujo de trabajo:**

1. Abrir RViz. Localizar el panel **Motion Planning** a la izquierda.
2. Establecer **Planning Group** a `arm` (o `gripper`).
3. En la pestaña **Start State**, elegir uno de:
   - `<current>` — usa las posiciones articulares reales actuales
   - Cualquier estado nombrado de la tabla anterior (p. ej., `up`, `rest`, `ready_near`)
4. En la pestaña **Goal State**, elegir el estado nombrado objetivo.
5. Hacer clic en **Plan**.
6. Revisar la trayectoria en RViz (fantasma animado del robot).
7. Hacer clic en **Execute** (o **Plan & Execute** para combinar los pasos 5–7).

**Flujos de trabajo comunes:**

| Desde      | Hasta              | Caso de uso                               |
|------------|--------------------|-------------------------------------------|
| `<current>`| `rest`             | Posición segura de reposo                 |
| `<current>`| `up`               | Configuración vertical/recta              |
| `<current>`| `ready_near`       | Preparar para recoger objeto cercano      |
| `<current>`| `ready_mid`        | Preparar para recoger objeto a media distancia |
| `<current>`| `ready_far`        | Preparar para recoger objeto lejano       |
| `<current>`| `overRightNearCan` | Posicionar sobre lata a la derecha        |
| `<current>`| `open`             | Abrir gripper (grupo de planificación: gripper) |
| `<current>`| `closed`           | Cerrar gripper (grupo de planificación: gripper) |

---

### 7.3 Usar Estados Nombrados vía el Tópico `joint_command` (nodo commander)

El nodo `commander` se suscribe a `/joint_command` para mover el brazo a una
configuración articular específica usando el grupo `arm` de MoveIt.

**Tópico:** `/joint_command`
**Tipo de mensaje:** `example_interfaces/msg/Float64MultiArray`

El array debe contener exactamente **4 valores**, uno por cada joint del grupo `arm`,
en el siguiente orden: `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex]`.

```bash
ros2 topic pub --once /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [<pan>, <lift>, <codo>, <muneca>]}"
```

**Ejemplo — Mover el brazo a la posición `up` (todos a cero):**
```bash
ros2 topic pub --once /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0]}"
```

**Ejemplo — Mover el brazo a `ready_near`:**
```bash
ros2 topic pub --once /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 1.5708, 1.5708]}"
```

**Ejemplo — Mover el brazo a `rest`:**
```bash
ros2 topic pub --once /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, -1.5708, 2.2689, 1.5708]}"
```

**Ejemplo — Mover el brazo a `ready_mid`:**
```bash
ros2 topic pub --once /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.3491, 1.5708, 1.2217]}"
```

**Ejemplo — Mover el brazo a `ready_far`:**
```bash
ros2 topic pub --once /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 1.5708, 0.0, 1.5708]}"
```

**Ejemplo — Mover el brazo a `overRightNearCan`:**
```bash
ros2 topic pub --once /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [-1.5708, 0.3665, 1.3963, 1.3963]}"
```

**Comportamiento esperado:** El nodo `commander` llama a MoveIt con `setJointValueTarget`,
planifica usando OMPL y ejecuta. La escala de velocidad y aceleración está al 100%.

---

### 7.4 Estados Nombrados vía `ros2 action` al move_group de MoveIt

El `move_group` de MoveIt expone una acción `MoveGroup`. Se pueden enviar goals de
estado nombrado directamente sin el nodo `commander`:

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup \
  "{request: {
     group_name: 'arm',
     motion_plan_request: {
       start_state: {is_diff: true},
       goal_constraints: [{
         name: 'rest',
         joint_constraints: [
           {joint_name: 'phantomx_pincher_arm_shoulder_pan_joint',  position: 0.0,     tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
           {joint_name: 'phantomx_pincher_arm_shoulder_lift_joint', position: -1.5708, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
           {joint_name: 'phantomx_pincher_arm_elbow_flex_joint',    position: 2.2689,  tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
           {joint_name: 'phantomx_pincher_arm_wrist_flex_joint',    position: 1.5708,  tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}
         ]
       }]
     },
     planning_options: {plan_only: false, look_around: false, replan: false}
  }}"
```

Establecer `plan_only: true` para solo calcular el plan sin ejecutarlo.

---

## 8. Todos los Tópicos, Servicios y Acciones

### 8.1 Tópicos

| Tópico               | Dirección   | Tipo de Mensaje                                        | Publicador/Suscriptor                  |
|----------------------|-------------|--------------------------------------------------------|----------------------------------------|
| `/joint_states`      | Publicado   | `sensor_msgs/msg/JointState`                           | `follow_joint_trajectory` (real) o `joint_state_broadcaster` (sim) |
| `/joint_command`     | Suscrito    | `example_interfaces/msg/Float64MultiArray`             | Nodo `commander`                       |
| `/open_gripper`      | Suscrito    | `example_interfaces/msg/Bool`                          | Nodo `commander`                       |
| `/pose_command`      | Suscrito    | `phantomx_pincher_interfaces/msg/PoseCommand`          | Nodo `commander`                       |
| `/tf`                | Publicado   | `tf2_msgs/msg/TFMessage`                               | `robot_state_publisher`                |
| `/robot_description` | Publicado   | `std_msgs/msg/String`                                  | `robot_state_publisher`                |

### 8.2 Action Servers

| Action Server                                                    | Tipo                                        | Nodo                                   |
|------------------------------------------------------------------|---------------------------------------------|----------------------------------------|
| `/joint_trajectory_controller/follow_joint_trajectory`           | `control_msgs/action/FollowJointTrajectory` | `pincher_follow_joint_trajectory` (real) o `joint_trajectory_controller` (sim) |
| `/gripper_trajectory_controller/follow_joint_trajectory`         | `control_msgs/action/FollowJointTrajectory` | `pincher_follow_joint_trajectory` (real) o `gripper_trajectory_controller` (sim) |
| `/move_action`                                                   | `moveit_msgs/action/MoveGroup`             | `move_group`                           |
| `/execute_trajectory`                                            | `moveit_msgs/action/ExecuteTrajectory`     | `move_group`                           |

### 8.3 Servicios Clave de MoveIt

| Servicio                             | Tipo                                         | Propósito                              |
|--------------------------------------|----------------------------------------------|----------------------------------------|
| `/plan_kinematic_path`               | `moveit_msgs/srv/GetMotionPlan`             | Planificar trayectoria (sin ejecutar)  |
| `/compute_ik`                        | `moveit_msgs/srv/GetPositionIK`             | Calcular cinemática inversa            |
| `/compute_fk`                        | `moveit_msgs/srv/GetPositionFK`             | Calcular cinemática directa            |
| `/get_planning_scene`                | `moveit_msgs/srv/GetPlanningScene`          | Consultar el estado de la escena de planificación |
| `/apply_planning_scene`              | `moveit_msgs/srv/ApplyPlanningScene`        | Modificar la escena de planificación   |

---

## 9. Referencia de Nodos

### 9.1 `commander` (`phantomx_pincher_commander_cpp`)

**Ejecutable:** `ros2 run phantomx_pincher_commander_cpp commander`

**Descripción:** Wrapper en C++ alrededor del `MoveGroupInterface` de MoveIt. Se suscribe
a tópicos de conveniencia y los traduce en solicitudes de movimiento a MoveIt.

**Suscripciones:**
- `/open_gripper` (`Bool`) → llama a `setNamedTarget("open")` o `setNamedTarget("closed")`
- `/joint_command` (`Float64MultiArray`, tamaño=4) → llama a `setJointValueTarget`
- `/pose_command` (`PoseCommand`) → llama a `setPoseTarget` o `computeCartesianPath`

**Configuración de MoveIt:**
- Escala de velocidad: 1.0 (100%)
- Escala de aceleración: 1.0 (100%)
- Tolerancia de posición: 0.01 m
- Tolerancia de orientación: 0.1 rad
- Tiempo de planificación: 5.0 s

---

### 9.2 `pincher_follow_joint_trajectory` (`pincher_control`)

**Ejecutable:** `ros2 run pincher_control follow_joint_trajectory`

**Descripción:** Driver de hardware para los servos AX-12A reales. Conecta las
trayectorias de MoveIt (en radianes/metros) con los comandos Dynamixel (en ticks).

**Parámetros:**

| Parámetro      | Por defecto         | Descripción                         |
|----------------|---------------------|-------------------------------------|
| `port`         | `/dev/ttyUSB0`      | Puerto serie                        |
| `baudrate`     | `1000000`           | Baudrate Dynamixel                  |
| `joint_prefix` | `phantomx_pincher_` | Prefijo de nombres de joints        |
| `moving_speed` | `200`               | Velocidad del servo (0–1023)        |
| `torque_limit` | `800`               | Límite de torque del servo (0–1023) |
| `gripper_id`   | `5`                 | ID Dynamixel del servo del gripper  |

**Publica:** `/joint_states` a 50 Hz con las posiciones reales de los servos.

**Fórmula Rad → Tick (joints del brazo):**
```
tick = 512 + grados(rad) × (1023 / 300)
```

**Fórmula Metros → Tick (gripper):**
```
t = (metros - 0.00375) / (0.01955 - 0.00375)
tick = round(t × 520)
```

---

### 9.3 `pincher_controller` (`pincher_control` — GUI)

**Ejecutable:** `ros2 run pincher_control control_servo`

**Descripción:** Control directo de Dynamixel con GUI de Tkinter. NO usa MoveIt.
Publica joint states solo para visualización en RViz.

---

### 9.4 `test_moveit` (`phantomx_pincher_commander_cpp`)

**Ejecutable:** `ros2 run phantomx_pincher_commander_cpp test_moveit`

**Descripción:** Ejemplo independiente en C++ que demuestra:
1. Meta de pose con planificación IK (comentado: meta nombrada, meta articular)
2. Trayectoria Cartesiana a través de 3 puntos de paso

**No está pensado para uso en producción** — sirve como tutorial/test de la API de MoveIt.

---

## 10. Tarjeta de Referencia Rápida

### Lanzar el Stack

```bash
# Simulación (recomendado para pruebas)
ros2 launch phantomx_pincher fake.launch.py

# Robot real
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=true
```

### Mover el Brazo a un Estado Nombrado (vía tópico joint_command)

```bash
# up → todos a cero
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# rest → posición recogida compacta
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.0, -1.5708, 2.2689, 1.5708]}"

# ready_near → brazo apuntando al frente, doblado por el codo
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.0, 0.0, 1.5708, 1.5708]}"

# ready_mid → brazo a distancia media
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.0, 0.3491, 1.5708, 1.2217]}"

# ready_far → brazo extendido lejos
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.0, 1.5708, 0.0, 1.5708]}"

# overRightNearCan → girado 90° a la derecha, alcanzando cerca
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray "{data: [-1.5708, 0.3665, 1.3963, 1.3963]}"
```

### Abrir / Cerrar Gripper (vía commander)

```bash
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: true}"   # abrir
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: false}"  # cerrar
```

### Mover a una Pose Cartesiana (vía commander)

```bash
ros2 topic pub --once /pose_command \
  phantomx_pincher_interfaces/msg/PoseCommand \
  "{x: 0.15, y: 0.0, z: 0.12, roll: 3.14159, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```

### Comando Directo de Motores (ticks, sin MoveIt)

```bash
# Mover los 5 motores al centro (512 ticks = 0 rad)
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[1,2,3,4,5] \
  -p goal_positions:=[512,512,512,512,512] \
  -p moving_speed:=150 -p delay:=3.0

# Abrir gripper (520 ticks)
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[5] -p goal_positions:=[520] -p moving_speed:=100 -p delay:=2.0

# Cerrar gripper (0 ticks)
ros2 run pincher_control control_servo --ros-args \
  -p dxl_ids:=[5] -p goal_positions:=[0] -p moving_speed:=100 -p delay:=2.0
```

### GUI Interactiva (sliders + RViz)

```bash
ros2 run pincher_control control_servo
```

---

## Apéndice — Definición del Mensaje `PoseCommand`

```
# phantomx_pincher_interfaces/msg/PoseCommand
float64 x             # metros, en el frame de planificación del brazo
float64 y             # metros
float64 z             # metros
float64 roll          # radianes
float64 pitch         # radianes
float64 yaw           # radianes
bool    cartesian_path  # si es true, usar interpolación Cartesiana en línea recta
```

## Apéndice — Referencia de Conversión del Gripper

```
Tipo de joint del gripper: PRISMÁTICO (metros, no radianes)

Calibración física:
  CERRADO: tick=0,   metros_por_dedo=0.00375  (7.5 mm total)
  ABIERTO: tick=520, metros_por_dedo=0.01955  (39.1 mm total)

Metros → Ticks:
  t    = (m - 0.00375) / (0.01955 - 0.00375)
  tick = clamp(round(t × 520), 0, 1023)

Ticks → Metros:
  t = tick / 520
  m = 0.00375 + t × (0.01955 - 0.00375)
```

## Apéndice — Referencia de Conversión de Joints del Brazo AX-12A

```
Rango de ticks: 0–1023
Centro:         512 ticks = 0 rad
Rango completo: 300° = 5.236 rad

Radianes → Ticks:
  tick = 512 + grados(rad) × (1023 / 300)
  tick = clamp(tick, 0, 1023)

Ticks → Radianes:
  grados = (tick - 512) × (300 / 1023)
  rad    = radianes(grados)
```
