# Guía ROS Basics — Parte 2


**Autor:** Johan Alejandro López Arias ([@ElJoho](https://github.com/ElJoho))


Continuación de la primera parte sobre publishers y subscribers. Aquí se cubren tres piezas que vas a necesitar para el laboratorio: leer teclas del terminal sin presionar Enter, detectar cuándo el robot terminó de moverse, y publicar comandos para mover el robot a posiciones predefinidas.

### Videos asociados

- [UNAL Phantom X Pincher: Capítulo 5 ROS2, Modificar terminal para utilizar teclado inmediato](https://www.youtube.com/watch?v=b5FdVRdaViI)
- [UNAL Phantom X Pincher: Capítulo 6 ROS2, Detectar movimiento de Phantom X Pincher](https://www.youtube.com/watch?v=ZREPMuBVMM4)
- [UNAL Phantom X Pincher: Capítulo 7 ROS2, Crear un publisher para mover el robot en rutina](https://www.youtube.com/watch?v=bQ8cChUb7i4)

### Scripts asociados

- [ejemplo_teclado.py](./ejemplo_teclado.py)
- [ejemplo_deteccion_movimiento.py](./ejemplo_deteccion_movimiento.py)
- [ejemplo_mover_robot.py](./ejemplo_mover_robot.py)

> **Nota sobre los scripts:** los archivos están en `guias/ROS2_basics/` con fines de referencia, pero en los videos se crean dentro del paquete `pincher_control`. Cuando sigas los videos vas a recrearlos ahí mismo.

---

## Parte 1 — Lectura inmediata de teclas (Capítulo 5)

### El problema

Normalmente, cuando escribes en un terminal y presionas una tecla, no pasa nada en tu programa hasta que oprimes **Enter**. Esto se debe a que el terminal está en modo **cooked** (cocinado): acumula los caracteres en un buffer, te deja editar con Backspace, interpreta Ctrl+C, y solo después de Enter envía la línea completa a tu programa.

Para un control tipo "presiona A para mover el robot a la posición A", esto no sirve. Necesitamos que cada tecla llegue al programa al instante.

### La solución: modo raw

Vamos a cambiar el terminal a modo **raw** (crudo) usando tres librerías:

| Librería   | Para qué sirve                                                                  |
|------------|---------------------------------------------------------------------------------|
| `sys`      | Acceso a `sys.stdin` (teclado) y `sys.stdout` (pantalla).                       |
| `termios`  | Envoltorio POSIX para configurar el terminal a bajo nivel.                      |
| `tty`      | Función `tty.setraw()` que activa el modo crudo en una sola línea.              |

> Nota: `termios` y `tty` solo funcionan en Linux y macOS, no en Windows.

### Código

```python
#!/usr/bin/env python3
import sys
import termios
import tty

TECLAS = {
    'a': 'te_amo',
    'b': 'te_odio',
    'c': 'te_ignoro',
    'd': 'NO_IGNORES_QUE_TE_IGNORO'
}

def leer_teclado():
    fd = sys.stdin.fileno()
    config_original = termios.tcgetattr(fd)

    print('Modo raw activo. Presiona a/b/c/d para probar, q para salir.\n')

    try:
        tty.setraw(fd)
        while True:
            tecla = sys.stdin.read(1)

            if tecla == 'q':
                print('\r\nSaliendo.')
                break

            if tecla in TECLAS:
                print(f'\r\nTecla: {tecla!r} significa que: {TECLAS[tecla]}')
            else:
                print(f'\r\nTecla desconocida: {tecla!r}')

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, config_original)
        print('Terminal restaurada.')

if __name__ == '__main__':
    leer_teclado()
```

### Puntos importantes

**El bloque `try`/`finally` es obligatorio.** Si el programa se cae con el terminal en modo raw y no restauramos la configuración, el terminal queda inutilizable y toca cerrarlo. El bloque `finally` se ejecuta pase lo que pase, así que ahí restauramos.

**`\r\n` en vez de `\n`.** En modo cooked, Linux convierte `\n` a `\r\n` automáticamente. En modo raw esa conversión está desactivada, así que tenemos que escribir el retorno de carro manual.

**`!r` en los f-strings.** Aplica `repr()` al valor, lo que muestra caracteres invisibles como Enter o Escape de forma legible (`'\r'`, `'\x1b'`). Útil para depurar teclas raras.

**No funciona Ctrl+C en modo raw.** Por eso usamos `q` como tecla de salida.

---

## Parte 2 — Detección de movimiento (Capítulo 6)

### El problema

Cuando le mandamos al robot ir a una posición, queremos saber cuándo terminó para poder enviarlo a la siguiente. Hay varias maneras de detectar esto:

1. Esperar la respuesta del action goal (`goal reached`). **Lo descartamos** porque a veces el goal no se alcanza (por errores, fricción, o porque alguien movió el robot manualmente).
2. Observar los topics que publica el robot y deducir el estado a partir de ellos. **Esta es la que usamos**, porque es robusta ante cualquier forma de mover el robot.

### El topic `/joint_states`

El paquete de control del Phantom publica continuamente el estado de cada motor en el topic `/joint_states` (mensaje `sensor_msgs/JointState`). Lo puedes verificar con:

```bash
ros2 topic list
ros2 topic echo /joint_states
```

Los campos relevantes del mensaje son:

| Campo       | Descripción                                              |
|-------------|----------------------------------------------------------|
| `name[]`    | Nombres de los joints (ej. `joint_1`, `joint_2`, ...)    |
| `position[]`| Posiciones angulares en radianes                         |
| `velocity[]`| Velocidades en rad/s (no siempre disponibles)            |
| `effort[]`  | Torque aplicado en cada joint                            |

### Estrategia

Hacemos un nodo que se suscribe a `/joint_states` y, en cada mensaje, calcula un valor `delta` que representa cuánto se está moviendo el robot. Si `delta` supera un umbral, el robot está en movimiento; si pasa un tiempo por debajo del umbral, el robot se detuvo.

Hay dos formas de calcular el `delta`, y elegimos automáticamente cuál usar:

- **Opción B (preferida):** usar directamente el campo `velocity` si trae valores reales. Es independiente de la frecuencia del topic.
- **Opción A (fallback):** comparar la posición actual contra la del frame anterior. Se usa cuando el firmware no reporta velocidad.

### Máquina de estados

Para evitar declarar prematuramente que "el robot se detuvo" cuando apenas está arrancando, usamos una máquina de estados con dos estados (REPOSO y MOVIENDO) y dos variables auxiliares:

- `_robot_moviendose` — bandera que dice si actualmente está en movimiento.
- `_lecturas_estables` — contador de frames consecutivos con `delta` pequeño. Solo se acumula si veníamos moviéndonos, y debe llegar a `LECTURAS_ESTABLES` para confirmar la detención.

```text
                   delta > umbral
                  ┌──────────────┐
                  ▼              │
   ┌─────────┐                ┌─────────┐
   │ REPOSO  │                │MOVIENDO │
   └─────────┘                └─────────┘
        ▲                         │
        │                         │
        └─── N frames quietos ────┘
```

### Umbrales

Los tres parámetros que controlan la sensibilidad están al inicio del script:

```python
UMBRAL_VELOCIDAD   = 0.005    # rad/s; usado si msg.velocity es confiable.
UMBRAL_MOVIMIENTO  = 0.0005   # rad entre frames; fallback por posición.
LECTURAS_ESTABLES  = 25       # frames consecutivos para confirmar detención (≈ 0.5 s a 50 Hz).
```

Estos valores se eligen por prueba y error y dependen del ruido del encoder. Si los bajas demasiado vas a tener falsos positivos por el ruido (el motor Dynamixel reporta variaciones del orden de 0.002 rad incluso quieto). Si los subes demasiado, los movimientos lentos no se detectan. Los valores propuestos son agresivos: si ves falsos "robot en movimiento" con el brazo quieto, súbelos.

### Pseudocódigo del callback

```text
FUNCIÓN _cb_joint_states(msg):

    # --- FASE 1: calcular delta y umbral ---

    SI msg.velocity tiene valores no-cero ENTONCES:
        delta  = velocidad absoluta más alta entre los joints
        umbral = UMBRAL_VELOCIDAD
        guardar msg.position en _ultimas_pos
    SI NO:
        SI _ultimas_pos es None ENTONCES:
            guardar msg.position y RETORNAR
        delta  = mayor diferencia posicional entre frame actual y anterior
        umbral = UMBRAL_MOVIMIENTO
        guardar msg.position en _ultimas_pos

    # --- FASE 2: máquina de estados ---

    SI delta > umbral ENTONCES:
        SI _robot_moviendose era False ENTONCES:
            LOG "robot en movimiento"
        _robot_moviendose = True
        _lecturas_estables = 0

    SI NO, SI _robot_moviendose es True ENTONCES:
        _lecturas_estables += 1
        SI _lecturas_estables ≥ LECTURAS_ESTABLES ENTONCES:
            _robot_moviendose = False
            _lecturas_estables = 0
            LOG "robot se detuvo"

    SI NO:
        # robot en reposo permanente, no hacer nada
        pasar
```

### Ejecución

```bash
# Terminal 1
ros2 launch phantom_bringup phantom_bringup.launch.py

# Terminal 2
ros2 run pincher_control ejemplo_deteccion_movimiento
```

Acuérdate de añadir el entry point en `setup.py`, verificar `sensor_msgs` en `package.xml` y compilar con `colcon build --packages-select pincher_control` antes de correrlo.

---

## Parte 3 — Mover el robot con un publisher (Capítulo 7)

### El problema

Hasta ahora sabemos detectar movimiento, pero no hemos comandado al robot desde código. En este capítulo creamos un nodo **publisher** que envía al robot a una secuencia de posiciones predefinidas.

### El topic `/named_target`

El controlador del Phantom escucha el topic `/named_target` (mensaje `std_msgs/String`), donde el campo `data` contiene el nombre de una pose declarada en el SRDF (`up`, `rest`, `ready_near`, etc.). El controlador se encarga de planificar y ejecutar el movimiento. Para más detalles sobre las poses disponibles, ver la guía de Motion Commands.

### Estrategia

El nodo:

1. Crea un publisher sobre `/named_target`.
2. Mantiene un índice que recorre una lista de poses.
3. Cada `ESPERA_SEGUNDOS` (suficiente para que el robot llegue), publica la siguiente pose.
4. Al terminar la lista, cancela el timer y cierra el nodo.

Para programar la publicación periódica usamos `create_timer(periodo, callback)`. El timer dispara automáticamente el callback en intervalos regulares.

### Código

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SECUENCIA = ['up', 'rest', 'ready_near', 'rest']
ESPERA_SEGUNDOS = 15.0


class MoverRobot(Node):

    def __init__(self):
        super().__init__('mover_robot')

        self.pub = self.create_publisher(String, '/named_target', 10)
        self._indice = 0

        self.timer = self.create_timer(ESPERA_SEGUNDOS, self._cb_timer)
        self._cb_timer()  # Dispara inmediatamente la primera pose.

        self.get_logger().info('Nodo mover_robot iniciado.')

    def _cb_timer(self):
        if self._indice >= len(SECUENCIA):
            self.timer.cancel()
            self.get_logger().info('Secuencia completada.')
            raise SystemExit

        posicion = SECUENCIA[self._indice]

        msg = String()
        msg.data = posicion
        self.pub.publish(msg)

        self.get_logger().info(
            f'[{self._indice + 1}/{len(SECUENCIA)}] Enviando: {posicion}'
        )
        self._indice += 1


def main(args=None):
    rclpy.init(args=args)
    node = MoverRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Puntos importantes

**Por qué `raise SystemExit` y no `return`.** El callback es invocado por `rclpy.spin()`. Un `return` solo sale del callback y devuelve el control al spin, que sigue esperando más callbacks indefinidamente. `SystemExit` es una excepción especial que `rclpy.spin()` captura para salir del bucle interno y devolver el control a `main()`, donde se ejecutan `destroy_node()` y `shutdown()`. Es equivalente a llamar a `sys.exit()`.

**`self.get_logger()`.** Es un método heredado de `Node` que devuelve el logger asociado al nodo. Las ventajas frente a `print()`: prefija automáticamente el mensaje con marca de tiempo, nivel y nombre del nodo; se puede filtrar por nivel (`debug`, `info`, `warn`, `error`, `fatal`); se ve en `rqt_console`; se publica en `/rosout`.

**Disparar el callback inmediatamente.** `create_timer` dispara la primera vez después de `ESPERA_SEGUNDOS`, no de inmediato. Llamamos `self._cb_timer()` manualmente al final del `__init__` para que la primera pose se envíe sin esperar.

**Ajustar `ESPERA_SEGUNDOS`.** Debe ser mayor que el tiempo que tarda el robot en alcanzar cada pose. Si la velocidad en MoveIt está al 100 %, 15 segundos sobra. Si bajaste la velocidad para movimientos suaves, sube este valor.

### Ejecución

```bash
# Terminal 1
ros2 launch phantom_bringup phantom_bringup.launch.py

# Terminal 2 (opcional: detector de movimiento del Capítulo 6)
ros2 run pincher_control ejemplo_deteccion_movimiento

# Terminal 3
ros2 run pincher_control ejemplo_mover_robot
```

Con el detector de movimiento corriendo en otra terminal, vas a ver los logs intercalados: el publisher anuncia "Enviando: rest", el detector confirma "Robot en movimiento", y un rato después "Robot se detuvo. Listo para el siguiente comando".
