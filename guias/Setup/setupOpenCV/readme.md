# Instalación de OpenCV en Ubuntu 24.04 (compilar desde el código fuente + módulos contrib)

---

## 0) Verificar Python + pip

```bash
python3 --version
pip3 --version
```

---

## 1) Actualizar el sistema

```bash
sudo apt update
sudo apt upgrade
```

---

## 2) (Opcional) Instalar el paquete de desarrollo de OpenCV de Ubuntu

```bash
sudo apt install libopencv-dev
```

---

## 3) Instalar dependencias de compilación

```bash
sudo apt install build-essential cmake git pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev
```

---

## 4) Descargar el código fuente de OpenCV + opencv_contrib

Crear una carpeta de trabajo.

```bash
mkdir -p ~/opencv_build
cd ~/opencv_build
```

Clonar OpenCV (el transcript referencia la URL oficial del repo).

```bash
git clone https://github.com/opencv/opencv.git
```

Clonar opencv_contrib (de `guideCommandsOpenCV.html`).

```bash
git clone https://github.com/opencv/opencv_contrib.git
```

---

## 5) Crear el directorio de compilación (build)


```bash
mkdir -p ~/opencv_build/opencv/build
cd ~/opencv_build/opencv/build
```

---

## 6) Configurar con CMake

El fragmento del transcript no muestra la línea exacta `cmake ...`, pero para compilar desde código fuente sí se necesita. Usa esta configuración común (también habilita los módulos contrib):

```bash
cmake -D CMAKE_BUILD_TYPE=Release       -D CMAKE_INSTALL_PREFIX=/usr/local       -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules       -D OPENCV_GENERATE_PKGCONFIG=ON       -D BUILD_EXAMPLES=ON       ..
```

---

## 7) Compilar + instalar
Esto es para raspberry pi 5, para computadores mas potentes pueden usar "make -j4" o "make -j6" depedendiendo de la cantidad de nucleos en el procesador de su computador y que RAM tengan.

```bash
make -j2
```


```bash
sudo make install
```

Recomendado después de instalar:

```bash
sudo ldconfig
```

---

## 8) Verificar la instalación

Ver la versión instalada con pkg-config.

```bash
pkg-config --modversion opencv4
```

Verificar que Python pueda importar OpenCV e imprimir la versión.

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

---