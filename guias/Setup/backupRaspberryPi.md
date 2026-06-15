# Crear un backup del sistema de la Raspberry Pi 5 con Disks (Ubuntu 24.04)


**Autor:** Johan Alejandro López Arias ([@ElJoho](https://github.com/ElJoho))


<details>
  <summary>
    <b>Video tutorial</b> — <a href="https://www.youtube.com/watch?v=gqHvcgRHNKI">Crear backup del sistema de la Raspberry Pi 5 usando Disks en Ubuntu 24.04</a>
  </summary>
</details>
<br>

Esta guía explica cómo crear una imagen de **backup** del sistema completo de la Raspberry Pi del kit del PhantomX Pincher usando el programa **Disks** de **Ubuntu 24.04**, comprimirla con **xz**, dividirla en varias partes para poder subirla a **GitHub** (sin superar el tamaño máximo por archivo de **Git LFS**), y publicarla en el repositorio.

Un backup completo te permite restaurar **todo** el sistema (Ubuntu, ROS 2, el workspace y la configuración) en una nueva tarjeta SD si la actual se corrompe o se daña.

---

## Requisitos previos

- Computador con **Ubuntu 24.04**.
- Lector de tarjetas o **adaptador USB → microSD**.
- La tarjeta microSD de la Raspberry Pi del kit.
- Espacio libre en disco suficiente (la imagen **sin comprimir** ocupa ~64 GB).
- `xz` y `split` (vienen preinstalados en Ubuntu).
- Git y Git LFS configurados (ver la guía de Setup principal).

> ⚠️ **Importante:** Apaga la Raspberry Pi correctamente y desenergízala **antes** de retirar la microSD, para no corromper la tarjeta.

---

## 1. Conectar la microSD al computador

1. Apaga y desenergiza la Raspberry Pi.
2. Retira la tarjeta microSD.
3. Conéctala al computador usando el **adaptador** (lector USB → microSD).

---

## 2. Crear la imagen con Disks

1. Abre el programa **Disks** (Discos) en Ubuntu.
2. Selecciona la tarjeta de la Raspberry Pi en la lista de la izquierda.
3. Haz clic en el menú de **tres puntos (⋮)** (esquina superior derecha) y elige **Create Disk Image…** (Crear imagen de disco).
4. Elige la carpeta donde guardar la imagen (por ejemplo una carpeta `images/`) y asígnale un nombre que incluya la fecha. En el video se usó:

   ```
   ubuntu24_04_date_2026.img
   ```

   > 💡 Incluir la fecha (`date_2026`) en el nombre ayuda a distinguir entre backups de distintas épocas.

5. Haz clic en **Start Creating…**.
6. Ingresa tu contraseña cuando se solicite.
7. Espera a que termine la copia. **Tarda un buen rato**; la imagen resultante pesa ~**64 GB**.

> **Nota:** El tamaño de la imagen corresponde al tamaño **total** de la tarjeta, no solo a lo usado. Por eso conviene comprimirla en el siguiente paso.

---

## 3. Comprimir la imagen con xz

La imagen de 64 GB es muy pesada para manejar o subir. Comprímela con **xz**, que la reduce aproximadamente **8 veces** (de ~64 GB a ~**8 GB**).

Abre una terminal en la carpeta donde está la imagen y ejecuta:

```bash
xz -T0 ubuntu24_04_date_2026.img
```

- **`xz`**: compresor con muy buena tasa de compresión, ideal para archivar.
- **`-T0`**: usa **todos los núcleos** del procesador para acelerar la compresión.

> 💡 Puedes escribir las primeras letras del nombre y presionar **Tab** para autocompletar.

Al terminar obtendrás el archivo:

```
ubuntu24_04_date_2026.img.xz
```

> ⚠️ Este proceso también **tarda bastante**. El tamaño final puede variar entre backups según cuántos paquetes y archivos tenga instalada la Raspberry.

---

> [!IMPORTANT]
> ## 🔒 De aquí en adelante: solo para el monitor o profesor de robótica
>
> Los pasos que siguen (**dividir la imagen, subirla a GitHub y administrar el backup en el repositorio**) están pensados **únicamente para el monitor o el profesor de robótica** encargado de mantener el repositorio del kit.
>
> Si eres **estudiante**, no necesitas realizar estos pasos. A ti solo te interesa saber cómo **descargar y restaurar** el backup en caso de que la tarjeta SD se dañe (ver la sección de restauración).

---

## 4. Dividir la imagen en partes para GitHub

Aunque ~8 GB ya es manejable, **un solo archivo supera el tamaño máximo por archivo** que admite GitHub. La solución es dividir el `.xz` en **7 partes** más pequeñas, de modo que cada una quede por debajo del límite de Git LFS.

```bash
split -n 7 --numeric-suffixes=1 -a 3 ubuntu24_04_date_2026.img.xz "unpi5_PXP_ubuntu24_04.img.xz.part-"
```

**¿Qué hace cada parte del comando?**

- **`split`**: divide un archivo en varios trozos.
- **`-n 7`**: genera exactamente **7 archivos** de tamaño similar.
- **`--numeric-suffixes=1`**: usa sufijos numéricos empezando en `1` (en vez de letras `aa`, `ab`…).
- **`-a 3`**: usa **3 dígitos** en el sufijo (`001`, `002`, …).
- **`"unpi5_PXP_ubuntu24_04.img.xz.part-"`**: prefijo de salida. Se usa **el mismo nombre que ya tienen los archivos en GitHub** para poder reemplazarlos directamente.

Esto genera:

```
unpi5_PXP_ubuntu24_04.img.xz.part-001
unpi5_PXP_ubuntu24_04.img.xz.part-002
unpi5_PXP_ubuntu24_04.img.xz.part-003
unpi5_PXP_ubuntu24_04.img.xz.part-004
unpi5_PXP_ubuntu24_04.img.xz.part-005
unpi5_PXP_ubuntu24_04.img.xz.part-006
unpi5_PXP_ubuntu24_04.img.xz.part-007
```

> 💡 El nombre de salida es distinto al del archivo comprimido **a propósito**: así las partes coinciden con las que ya están en el repositorio y se reemplazan sin cambiar de nombre (los archivos del repo se mantienen estables, solo cambia su contenido en cada backup).

---

## 5. Subir las partes a GitHub

1. Asegúrate de que las partes se rastreen con **Git LFS** (solo hace falta configurarlo una vez por repositorio):

   ```bash
   git lfs track "*.img.xz.part-*"
   git add .gitattributes
   ```

2. Copia las **7 partes** a la carpeta del repositorio, **reemplazando** las anteriores.
3. Verifica los cambios:

   ```bash
   git status
   ```

   Deberías ver las 7 partes marcadas como modificadas.

4. Añade los archivos (por su tamaño, puede tardar un poco):

   ```bash
   git add .
   ```

5. Crea el commit describiendo el backup:

   ```bash
   git commit -m "Backup imagen Raspberry Pi 5 - Ubuntu 24.04 (junio 2026)"
   ```

6. Sube los cambios al repositorio:

   ```bash
   git push origin main
   ```

Al hacer `git push`, GitHub subirá las 7 partes vía Git LFS. El proceso puede tardar según tu conexión.

---

## 6. Restaurar el backup (resumen)

La restauración completa (descargar las partes, **concatenarlas** y **grabar** la imagen en una microSD) se cubre en un video aparte. El proceso se muestra en **Windows** para los estudiantes que lo necesiten, aunque en **Linux** es más sencillo.

En Linux, reconstruir el `.xz` es tan simple como concatenar las partes en orden:

```bash
cat unpi5_PXP_ubuntu24_04.img.xz.part-* > unpi5_PXP_ubuntu24_04.img.xz
```

> 💡 El `*` ordena las partes alfabéticamente; como los sufijos llevan ceros (`001`, `002`…), el orden coincide con el correcto.

Después se graba la imagen en la microSD con un programa como **Balena Etcher**, y tendrás el backup completo funcionando en la Raspberry Pi.

---

## Resumen rápido

| Paso | Comando / Acción |
|------|------------------|
| Crear imagen | **Disks → ⋮ → Create Disk Image…** (≈64 GB) |
| Comprimir | `xz -T0 ubuntu24_04_date_2026.img` (≈8 GB) |
| Dividir en 7 partes | `split -n 7 --numeric-suffixes=1 -a 3 <img.xz> "unpi5_PXP_ubuntu24_04.img.xz.part-"` |
| Rastrear con LFS | `git lfs track "*.img.xz.part-*"` |
| Subir | `git add . && git commit -m "..." && git push origin main` |
| Reconstruir (Linux) | `cat *.img.xz.part-* > unpi5_PXP_ubuntu24_04.img.xz` |
| Grabar en SD | **Balena Etcher** |