# Conexión Remota a la Raspberry Pi con VS Code via SSH


**Autor:** Johan Alejandro López Arias ([@ElJoho](https://github.com/ElJoho))


<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=o4gKgJH7CwI">Como usar visual studio code y remote SSH con raspberry pi</a>
  </summary>
</details>
<br>

Esta guía explica cómo conectarse a la Raspberry Pi desde VS Code en tu PC usando SSH, cómo transferir archivos, y cómo usar la terminal de la Pi directamente desde VS Code.

---

## Datos de conexión

| Campo | Valor |
|-------|-------|
| **Usuario** | `unpi` |
| **IP** | `192.168.10.2` |
| **Contraseña** | `unpi` |

---

## Requisitos previos

### En la Raspberry Pi
Primero, verifica la IP de la Raspberry Pi abriendo una terminal en la Pi y ejecutando:

```bash
# Ver la IP de la Pi
hostname -I
```

Anota la IP que aparece (por ejemplo `192.168.10.2`). Luego, verifica que el servidor SSH esté activo:

```bash
sudo systemctl status ssh
```

Si no está corriendo, instálalo y actívalo:

```bash
sudo apt install openssh-server -y # instalacion
sudo systemctl enable --now ssh # activacion
```

### En tu PC
- Tener **VS Code** instalado.
- Instalar la extensión **Remote - SSH** (publisher: Microsoft):
  1. Abre VS Code.
  2. Ve a Extensiones (`Ctrl+Shift+X`).
  3. Busca `Remote - SSH` e instálala.

---

## Primera conexión (solo se hace una vez)

1. Abre la **Paleta de Comandos** con `Ctrl+Shift+P`.
2. Escribe `Remote-SSH: Connect to Host` y selecciónalo.
3. Haz clic en **Add New SSH Host...**.
4. Escribe el siguiente comando SSH:

```
ssh unpi@192.168.10.2
```

5. Presiona Enter y guarda la configuración en el archivo por defecto (`~/.ssh/config`).
6. Aparecerá una notificación — haz clic en **Connect**.
7. Selecciona el sistema operativo de la Pi: **Linux**.
8. Ingresa la contraseña cuando se te pida: `unpi`.
9. VS Code instalará automáticamente su servidor remoto en la Pi (solo ocurre la primera vez, tarda un minuto).

Cuando la conexión sea exitosa, verás en la esquina inferior izquierda de VS Code un badge verde que dice:

```
>< SSH: 192.168.10.2
```

---

## Reconectarse después de cerrar VS Code

Una vez configurada la conexión, **no es necesario repetir el proceso**. Para volver a conectarse:

1. Haz clic en el botón verde **`><`** en la esquina inferior izquierda de VS Code.
2. Selecciona **Connect to Host...**.
3. Elige **`192.168.10.2`** de la lista.
4. Ingresa la contraseña `unpi` si se solicita.

Listo — VS Code recuerda la configuración permanentemente.

> **Tip:** Para evitar tener que ingresar la contraseña cada vez, puedes configurar autenticación por clave SSH (ver sección al final de esta guía).

---

## Abrir carpetas en la Pi desde VS Code

Una vez conectado, ve a **Archivo → Abrir Carpeta** (`Ctrl+K Ctrl+O`) y navega por el sistema de archivos de la Raspberry Pi como si fuera local. Todos los archivos que edites se guardan directamente en la Pi.

---

## Usar la terminal de la Raspberry Pi desde VS Code

Con la conexión SSH activa, puedes abrir una terminal que corre **directamente en la Pi**:

- Atajo de teclado: **Ctrl + `** (tecla backtick/acento grave)
- O ve al menú: **Terminal → New Terminal**

Cualquier comando que escribas en esa terminal se ejecuta en la Raspberry Pi, no en tu PC. Por ejemplo:

```bash
# Ver la IP de la Pi
hostname -I

# Actualizar paquetes
sudo apt update

# Ejecutar un nodo de ROS2
ros2 run <paquete> <nodo>

# Navegar al workspace de ROS2
cd ~/ros2/KIT_Phantom_X_Pincher_ROS2/phantom_ws
```

Puedes abrir **múltiples terminales** simultáneamente desde el mismo VS Code usando el botón **+** en el panel de terminal.

---

## Transferir archivos STL (y otros) a la Pi

### Opción 1 — Arrastrar y soltar en VS Code (más fácil)

Con la conexión SSH activa y una carpeta abierta en el Explorer de VS Code, simplemente arrastra archivos desde tu explorador de archivos local al panel del Explorer de VS Code. El archivo se copia directamente a la Pi.

### Opción 2 — SCP desde la terminal de tu PC

Abre una terminal **en tu PC** (no en VS Code) y ejecuta:

```bash
# Copiar un archivo
scp modelo.stl unpi@192.168.10.2:/home/unpi/modelos/

# Copiar una carpeta completa
scp -r ./carpeta_stl/ unpi@192.168.10.2:/home/unpi/modelos/
```

### Opción 3 — WinSCP o FileZilla (interfaz gráfica, Windows)

Usa los datos de conexión (IP: `192.168.10.2`, usuario: `unpi`, contraseña: `unpi`, protocolo: SFTP) para conectarte con WinSCP o FileZilla y transferir archivos con interfaz gráfica.

---

## Opcional: Conexión sin contraseña (clave SSH)

Para no tener que escribir la contraseña cada vez que conectas VS Code:

En una terminal de tu PC ejecuta:

```bash
ssh-keygen -t ed25519   # Omitir si ya tienes una clave generada
ssh-copy-id unpi@192.168.10.2
```

Ingresa la contraseña `unpi` una última vez. Después de esto, VS Code se conectará automáticamente sin pedirte contraseña.

---

## Notas

- **NoMachine y VS Code SSH son completamente compatibles** — puedes tener ambas conexiones activas al mismo tiempo sin interferencia. Usan protocolos y puertos distintos.
- La extensión Remote-SSH instala un servidor ligero en la Pi la primera vez (`~/.vscode-server/`). Las conexiones posteriores son instantáneas.
- Las extensiones de VS Code (Python, ROS, etc.) deben instalarse "en el host remoto" desde VS Code — aparecerá el botón **Install in SSH: 192.168.10.2** en lugar del botón de instalación normal.
