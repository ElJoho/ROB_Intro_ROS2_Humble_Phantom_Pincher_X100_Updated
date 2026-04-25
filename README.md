# KIT PHANTOM X PINCHER

**Autor:** Johan Alejandro López Arias ([@ElJoho](https://github.com/ElJoho))


**Director:** Pedro Fabián Cárdenas Herrera 

---

## Introduccion al Kit fisico

Se recomienda ver los siguientes videos introductorios al kit fisico:

<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=fg1AXdiOE6Y">Introducción al Kit fisico del robot</a>
  </summary>
</details>
<br>

<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=txS73u_q9bY">Como organizar el Kit en su contenedor</a>
  </summary>
</details>
<br>

---

## Setup

Para observar la guía de cómo hacer la instalación de ROS2 en Ubuntu 24.04 LTS puede dirigirse al siguiente archivo:

[Guia: Setup de ROS2](guias/Setup/readme.md)

### Conexión con Raspberry Pi para ver pantalla

Para observar la pantalla de la Raspberry Pi necesita realizar una conexión usando un cable ethernet y un programa conocido como NoMachine. El proceso de cómo hacerlo puede verlo en la guía en el siguiente enlace:

[Guia: Conexión NoMachine](guias/Setup/conexionNoMachine.md)

<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=30KEEl1uhBM">Como conectarse a las raspberry pi usando NoMachine</a>
  </summary>
</details>
<br>

### Probar el funcionamiento del robot usando Dynamixel Wizard 2.0

Para probar el funcionamiento del robot se puede usar el programa Dynamixel Wizard que ya viene instalado en la raspberry pi. No se cuenta con una guia pero si con un video mostrando el funcionamiento del programa y como se prueba el robot con Dynamixel Wizard 2.0

<details>
  <summary>
    <b>Video tutorial</b> — <a https://www.youtube.com/watch?v=VNw8zVJwRWo">Como usar Dynamixel Wizard 2.0</a>
  </summary>
</details>
<br>


### Conexión SSH en VS Code

Es posible usar Visual Studio Code para editar y subir archivos a la Raspberry Pi usando una extensión de SSH. El proceso para realizar esto se encuentra en la siguiente guía:


[Guia: Configuración SSH en VS Code](guias/Setup/sshVScode.md)

<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=30KEEl1uhBM">Como conectarse a las raspberry pi usando NoMachine</a>
  </summary>
</details>
<br>

---

### Añadiendo elementos al robot

<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=CDmSgoLtIdU">Como mover el robot (Mottion commands)</a>
  </summary>
</details>
<br>

En la siguiente guia se encuentra una serie de pasos para añadir elementos xacro al robot para su visualizaci´on en rviz. Adicionalmente hay una serie de 10 videos explicando las distintas maneras en que se pueden añadir elementos para modificar la plataforma del robot o al brazo robotico. Adicionalmente se explica como usar la ventosa en el video final.

[Guia: Añadir elementos xacro al robot](guias/URDF/addingXacroElements.md)

---

## MoveIt

### Como mover el robot

Para mover el robot puede leer la guia o ver el video que se muestran a continuacion:

[Guia: Comandos de Movimiento](guias/Moveit/MOTION_COMMANDS.md)


<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=CDmSgoLtIdU">Como mover el robot (Mottion commands)</a>
  </summary>
</details>
<br>

### Calibración del gripper

[Guia: Calibración del gripper](guias/Setup/guia_calibracion_gripper.md)