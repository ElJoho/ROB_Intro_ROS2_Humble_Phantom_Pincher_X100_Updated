# Guía: Calibración de la Apertura Máxima del Gripper

## Requisitos previos

- El bringup del robot **NO debe estar corriendo** durante este proceso.
  El puerto `/dev/ttyUSB0` debe estar libre para que los comandos funcionen.
- Tener el SDK de Dynamixel instalado (`pip install dynamixel-sdk`).

---

## Paso 1 — Apagar el torque del gripper y moverlo manualmente a la posición cerrada

Ejecuta el siguiente comando para apagar el torque del servo 5 (gripper):

```bash
python3 -c "
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler('/dev/ttyUSB0')
port.openPort()
port.setBaudRate(1000000)
packet = PacketHandler(1.0)
packet.write1ByteTxRx(port, 5, 24, 0)
print('Gripper apagado')
port.closePort()
"
```

Con el torque apagado, **mueve el gripper a mano hasta la posición completamente cerrada**
(los dedos juntos lo máximo posible).

---

## Paso 2 — Leer los ticks actuales (posición cerrada)

```bash
python3 -c "
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler('/dev/ttyUSB0')
port.openPort()
port.setBaudRate(1000000)
packet = PacketHandler(1.0)
tick, comm, err = packet.read2ByteTxRx(port, 5, 36)
print(f'Tick: {tick}  ({tick * 300.0 / 1023.0:.1f} grados)')
port.closePort()
"
```

> **Anota este valor. Ese es tu `GRIPPER_CLOSED_TICKS`.**

---

## Paso 3 — Encender el torque y mover de 50 en 50 ticks

A partir del valor leído en el paso 2, enciende el torque y mueve el gripper
sumando 50 ticks cada vez. Reemplaza `XXX` por el tick deseado en cada intento.

**Comando para mover a un tick específico:**

```bash
python3 -c "
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler('/dev/ttyUSB0')
port.openPort()
port.setBaudRate(1000000)
packet = PacketHandler(1.0)
packet.write1ByteTxRx(port, 5, 24, 1)   # encender torque
packet.write2ByteTxRx(port, 5, 30, XXX) # mover a tick XXX
port.closePort()
"
```

**Después de cada movimiento, lee la posición actual para confirmar:**

```bash
python3 -c "
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler('/dev/ttyUSB0')
port.openPort()
port.setBaudRate(1000000)
packet = PacketHandler(1.0)
tick, comm, err = packet.read2ByteTxRx(port, 5, 36)
print(f'Tick: {tick}  ({tick * 300.0 / 1023.0:.1f} grados)')
port.closePort()
"
```

Ejemplo de secuencia si el cerrado está en tick 0:

| Intento | Tick objetivo | Acción      |
|---------|--------------|-------------|
| 1       | 50           | `XXX = 50`  |
| 2       | 100          | `XXX = 100` |
| 3       | 150          | `XXX = 150` |
| 4       | 200          | `XXX = 200` |
| ...     | ...          | Continuar de 50 en 50 |

---

## Paso 4 — Afinar de 10 en 10 ticks al acercarse al máximo

Cuando el gripper esté casi en su apertura máxima (los dedos en su límite
físico), reduce el incremento a **10 ticks** para encontrar el límite exacto
sin forzar el mecanismo.

| Intento | Tick objetivo             | Acción                 |
|---------|--------------------------|------------------------|
| N       | valor\_anterior + 10     | Continuar de 10 en 10  |
| N+1     | valor\_anterior + 20     | Continuar de 10 en 10  |
| ...     | ...                      | Detener en límite físico |

> El límite físico es cuando los dedos no avanzan más o el servo
> empieza a forzar. **No sobrepasar ese punto.**

> **Anota el último tick en el que el gripper se mueve sin forzar.
> Ese es tu `GRIPPER_OPEN_TICKS`.**

---

## Paso 5 — Medir la apertura física

Con el gripper en la posición máxima abierta:
1. Mide la distancia total entre los dedos con un calibrador (en mm).
2. Divide ese valor entre 2 → eso es **`GRIPPER_OPEN_M`** (en metros).

Con el gripper en la posición mínima cerrada:
1. Mide la distancia total entre los dedos.
2. Divide entre 2 → eso es **`GRIPPER_CLOSED_M`** (en metros).

---

## Archivos a modificar para actualizar la calibración

Una vez obtenidos los nuevos valores, actualiza los siguientes **4 archivos**:

---

### Archivo 1 — Driver de hardware (nodo ROS 2)

**Ruta dentro de `src`:**
```
pincher_control/pincher_control/follow_joint_trajectory_node.py
```

**Líneas a cambiar:**
```python
GRIPPER_OPEN_TICKS   = 520     # ← reemplazar con tu GRIPPER_OPEN_TICKS
GRIPPER_CLOSED_TICKS = 0       # ← reemplazar con tu GRIPPER_CLOSED_TICKS
GRIPPER_OPEN_M       = 0.01955 # ← reemplazar con tu GRIPPER_OPEN_M (metros por dedo)
GRIPPER_CLOSED_M     = 0.00375 # ← reemplazar con tu GRIPPER_CLOSED_M (metros por dedo)
```

> Estos valores controlan la conversión ticks ↔ metros en el hardware real.

---

### Archivo 2 — Límites de joints para MoveIt

**Ruta dentro de `src`:**
```
phantomx_pincher_moveit_config/config/joint_limits.yaml
```

**Líneas a cambiar** (las mismas para `finger1` y `finger2`):
```yaml
min_position: 0.00375  # ← reemplazar con GRIPPER_CLOSED_M
max_position: 0.01955  # ← reemplazar con GRIPPER_OPEN_M
```

> MoveIt usa estos límites para validar el estado inicial y el goal.
> Si el valor reportado por el hardware está fuera de este rango,
> MoveIt rechaza la planificación (`CheckStartStateBounds`).

---

### Archivo 3 — Estados nombrados de MoveIt (open / closed)

**Ruta dentro de `src`:**
```
phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf
```

**Líneas a cambiar:**
```xml
<group_state group="gripper" name="open">
  <joint name="phantomx_pincher_gripper_finger1_joint" value="0.01955"/>  <!-- ← GRIPPER_OPEN_M -->
  <joint name="phantomx_pincher_gripper_finger2_joint" value="0.01955"/>  <!-- ← GRIPPER_OPEN_M -->
</group_state>
<group_state group="gripper" name="closed">
  <joint name="phantomx_pincher_gripper_finger1_joint" value="0.00375"/>  <!-- ← GRIPPER_CLOSED_M -->
  <joint name="phantomx_pincher_gripper_finger2_joint" value="0.00375"/>  <!-- ← GRIPPER_CLOSED_M -->
</group_state>
```

> Estos son los goals que aparecen en RViz cuando seleccionas
> "open" o "closed" en el panel de Motion Planning.

---

### Archivo 4 — URDF del robot

**Ruta dentro de `src`:**
```
phantomx_pincher_description/urdf/phantomx_pincher.urdf
```

**Líneas a cambiar** (las mismas para `finger1_joint` y `finger2_joint`):
```xml
<limit effort="30" lower="0.00375" upper="0.01955" velocity="0.01"/>
<!--                       ↑                 ↑
                    GRIPPER_CLOSED_M    GRIPPER_OPEN_M              -->
```

> El URDF define los límites físicos que usa MoveIt como referencia base.
> Si no se actualiza, MoveIt puede rechazar estados válidos por estar
> "fuera de los límites" del URDF.

---

## Resumen de valores a mantener sincronizados

| Valor                 | Descripción                          | Archivos donde aparece                                                                              |
|-----------------------|--------------------------------------|-----------------------------------------------------------------------------------------------------|
| `GRIPPER_OPEN_TICKS`  | Tick del servo cuando está abierto   | `follow_joint_trajectory_node.py`                                                                   |
| `GRIPPER_CLOSED_TICKS`| Tick del servo cuando está cerrado   | `follow_joint_trajectory_node.py`                                                                   |
| `GRIPPER_OPEN_M`      | Metros por dedo en posición abierta  | `follow_joint_trajectory_node.py`, `joint_limits.yaml`, `phantomx_pincher.srdf`, `phantomx_pincher.urdf` |
| `GRIPPER_CLOSED_M`    | Metros por dedo en posición cerrada  | `follow_joint_trajectory_node.py`, `joint_limits.yaml`, `phantomx_pincher.srdf`, `phantomx_pincher.urdf` |

> **Importante:** los 4 archivos deben tener siempre los mismos valores
> de `GRIPPER_OPEN_M` y `GRIPPER_CLOSED_M`. Si se actualiza uno, se
> deben actualizar todos para que el sistema sea consistente.
