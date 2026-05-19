# Edición remota con la extensión SFTP de VS Code

**Autor:** Johan Alejandro López Arias ([@ElJoho](https://github.com/ElJoho))

Esta guía explica cómo configurar la extensión [SFTP (Natizykunk)](https://marketplace.visualstudio.com/items?itemName=Natizykunk.sftp)
para editar el workspace de la Raspberry Pi directamente desde VS Code en Windows.

<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=CL0vpqbWdjM">UNAL PhantomX Pincher: Como usar visual studio code SFTP para conectarse a raspberry pi</a>
  </summary>
</details>
<br>

---

## Datos de conexión

| Campo | Valor |
|-------|-------|
| **Usuario** | `unpi` |
| **IP** | `192.168.10.2` |
| **Contraseña** | `unpi` |

---

## Requisitos previos

### Verificar la IP de la Raspberry Pi

Abre una terminal en la Pi y ejecuta:

```bash
hostname -I
```

Anota la IP que aparece (por ejemplo `192.168.10.2`) y úsala en el campo `host` del `sftp.json`.

### Verificar que SSH está activo

```bash
sudo systemctl status ssh
```

Si no está corriendo, instálalo y actívalo:

```bash
sudo apt install openssh-server -y
sudo systemctl enable --now ssh
```

### Verificar la conexión desde Windows

Antes de configurar la extensión, comprueba que puedes conectarte por SSH desde PowerShell:

```powershell
ssh unpi@192.168.10.2
```

Si la conexión funciona, la extensión SFTP también funcionará — usa el mismo protocolo.

---

## Cómo funciona

Editas una **copia local** del proyecto en Windows. Cada vez que guardas un archivo (`Ctrl+S`),
la extensión lo sube automáticamente a la Pi por SFTP. No se monta ninguna unidad de red —
la Pi solo usa el demonio SSH (`sshd`) que ya está corriendo.

```
Windows                              Raspberry Pi
───────                              ────────────
VS Code (edita copia local)          /home/unpi/ros2/KIT_Phantom_X_Pincher_ROS2/
         │                                    ▲
         └──── Ctrl+S → upload ───────────────┘
```

---

## Instalación

1. En VS Code (`Ctrl+Shift+X`): buscar **SFTP** (autor: **Natizykunk**) → **Install**.
2. Abre la carpeta local donde quieres trabajar en Windows (puede ser una carpeta vacía).
3. `Ctrl+Shift+P` → `SFTP: Config` — se crea automáticamente el archivo `.vscode/sftp.json`.

---

## Configuración del `sftp.json`

Reemplaza el contenido de `.vscode/sftp.json` con lo siguiente:

```json
{
    "name": "Raspberry Pi",
    "host": "192.168.10.2",
    "protocol": "sftp",
    "port": 22,
    "username": "unpi",
    "password": "unpi",
    "remotePath": "/home/unpi/ros2/KIT_Phantom_X_Pincher_ROS2",
    "uploadOnSave": true,
    "useTempFile": false,
    "openSsh": false,
    "ignore": [
        ".vscode",
        ".git",
        ".DS_Store",
        "phantom_ws/build/**",
        "phantom_ws/install/**",
        "phantom_ws/log/**",
        "**/.claude",
        "**/.claude/**",
        "**/__pycache__/**",
        "**/*.pyc"
    ]
}
```

### Descripción de los campos principales

| Campo | Valor | Descripción |
|-------|-------|-------------|
| `host` | `192.168.10.2` | IP de la Raspberry Pi |
| `remotePath` | `/home/unpi/ros2/...` | Ruta raíz del proyecto en la Pi |
| `uploadOnSave` | `true` | Sube el archivo al guardar con `Ctrl+S` |
| `useTempFile` | `false` | Escribe directamente (sin archivo temporal) |

### Por qué se ignoran esas carpetas

| Patrón | Razón |
|--------|-------|
| `.vscode` | Contiene `sftp.json` con la contraseña en texto plano — **nunca subir** |
| `.git` | Historial git completo, no necesita duplicarse en la Pi |
| `phantom_ws/build/**` | Generado por `colcon build` — pesado e innecesario |
| `phantom_ws/install/**` | Artefactos de build — igual |
| `phantom_ws/log/**` | Logs de colcon — irrelevantes |
| `**/.claude` y `**/.claude/**` | Configuración y memoria de Claude Code — ver nota |
| `**/__pycache__/**` | Caché de Python compilado, se regenera automáticamente |
| `**/*.pyc` | Archivos `.pyc` sueltos — misma razón |

> **Nota sobre `.claude`:** esta carpeta vive en `phantom_ws/src/.claude`, no en la raíz.
> El patrón `dir/**` solo aplica en el nivel raíz del `remotePath`. Para ignorar una carpeta
> a **cualquier profundidad** se necesitan dos patrones: `**/.claude` (oculta el directorio)
> y `**/.claude/**` (oculta el contenido).

#### Opcional — ROS 2 bags

Si en algún momento grabas bags, agrégalos porque pueden ocupar varios GB:

```json
"**/*.db3",
"**/*.mcap",
"**/rosbag2_*/**"
```

---

## Primer uso — bajar los archivos de la Pi

Antes de empezar a editar, descarga el proyecto completo de la Pi a tu carpeta local:

```
Ctrl+Shift+P → SFTP: Download Project
```

A partir de ahí, edita localmente y guarda con `Ctrl+S` — cada archivo se sube solo.

---

## Verificar que funciona

1. Abre `View → Output` y selecciona **SFTP** en el menú desplegable.
2. Guarda cualquier archivo dentro de `src/`.
3. Debe aparecer una línea como:

```
[info] Uploading /ruta/al/archivo.py
[info] Upload done.
```

Los patrones del campo `ignore` se aplican inmediatamente al guardar `sftp.json`, sin
reiniciar VS Code.

---

## Flujo de trabajo diario

```
1. Editar en VS Code (Windows)  →  Ctrl+S  →  archivo sube a la Pi automáticamente
2. Terminal SSH  →  colcon build / ros2 run / ros2 launch
3. Si algo cambia en la Pi  →  Ctrl+Shift+P → SFTP: Sync Remote → Local
```

---

## Limitación principal

Si un archivo **cambia en la Pi** (por ejemplo, archivos generados por `colcon`) **no se
descarga automáticamente** a Windows. Para sincronizar de vuelta:

```
Ctrl+Shift+P → SFTP: Download Project
             → SFTP: Sync Remote → Local
```

---

## Recursos

- [SFTP (Natizykunk) en VS Code Marketplace](https://marketplace.visualstudio.com/items?itemName=Natizykunk.sftp)
- [Repositorio de la extensión — GitHub](https://github.com/Natizykunk/vscode-sftp)
