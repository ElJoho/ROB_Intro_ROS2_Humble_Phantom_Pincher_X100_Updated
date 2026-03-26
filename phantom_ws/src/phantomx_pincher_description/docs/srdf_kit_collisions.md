# Actualizar colisiones del kit en el SRDF

## Contexto

El SRDF define pares de links cuyas colisiones están desactivadas (`<disable_collisions>`).
Cuando se añaden o eliminan links en `kit.xacro`, el SRDF debe actualizarse para reflejar
la nueva estructura. Este documento cubre dos cambios pendientes:

1. **Eliminar** las entradas de `manijaTrasera_link` y `manijaDelantera_link` — ya no son
   links independientes; su geometría de colisión fue absorbida dentro de `baseFija_link`.
2. **Añadir** las entradas para los cuatro links nuevos del kit que aún no tienen colisiones
   desactivadas: `soporteCamara_link`, `raspberry_link`, `canastilla_link`, `electronics_link`.

Los cambios deben aplicarse en **dos archivos**:

| Archivo | Ruta |
|---|---|
| SRDF directo | `phantom_ws/src/phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf` |
| Fuente xacro | `phantom_ws/src/phantomx_pincher_moveit_config/srdf/phantomx_pincher.xacro` |

Ambos archivos tienen las mismas entradas de colisiones del kit, solo cambia el prefijo del
brazo (`phantomx_pincher_arm_base_link` en el `.srdf` vs `${prefix}arm_base_link` en el
`.xacro`). Cada sección de abajo indica qué borrar y qué pegar en cada archivo.

---

## Parte 1 — Eliminar `manijaTrasera_link` y `manijaDelantera_link`

### En `phantomx_pincher.srdf`

Borrar las siguientes 18 líneas (están entre las líneas ~400 y ~465 del archivo):

```xml
  <disable_collisions link1="baseFija_link" link2="manijaTrasera_link" reason="Never"/>
  <disable_collisions link1="baseFija_link" link2="manijaDelantera_link" reason="Never"/>
  <disable_collisions link1="caseBaseRobot_link" link2="manijaTrasera_link" reason="Never"/>
  <disable_collisions link1="caseBaseRobot_link" link2="manijaDelantera_link" reason="Never"/>
  <disable_collisions link1="zonaRecoleccion_link" link2="manijaTrasera_link" reason="Never"/>
  <disable_collisions link1="zonaRecoleccion_link" link2="manijaDelantera_link" reason="Never"/>
  <disable_collisions link1="airpump_link" link2="manijaTrasera_link" reason="Never"/>
  <disable_collisions link1="airpump_link" link2="manijaDelantera_link" reason="Never"/>
  <!-- manijaTrasera_link vs others -->
  <disable_collisions link1="manijaTrasera_link" link2="manijaDelantera_link" reason="Never"/>
  <disable_collisions link1="manijaTrasera_link" link2="canecaLateralIzquierda_link" reason="Never"/>
  <disable_collisions link1="manijaTrasera_link" link2="canecaLateralDerecha_link" reason="Never"/>
  <disable_collisions link1="manijaTrasera_link" link2="canecaFrontalDerecha_link" reason="Never"/>
  <disable_collisions link1="manijaTrasera_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
  <!-- manijaDelantera_link vs others -->
  <disable_collisions link1="manijaDelantera_link" link2="canecaLateralIzquierda_link" reason="Never"/>
  <disable_collisions link1="manijaDelantera_link" link2="canecaLateralDerecha_link" reason="Never"/>
  <disable_collisions link1="manijaDelantera_link" link2="canecaFrontalDerecha_link" reason="Never"/>
  <disable_collisions link1="manijaDelantera_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
  <disable_collisions link1="phantomx_pincher_arm_base_link" link2="manijaTrasera_link" reason="Never"/>
  <disable_collisions link1="phantomx_pincher_arm_base_link" link2="manijaDelantera_link" reason="Never"/>
```

También borrar estas dos líneas del comentario de inventario (alrededor de la línea 386):

```
        manijaTrasera_link
        manijaDelantera_link
```

### En `phantomx_pincher.xacro`

Mismas entradas pero con `${prefix}` en el link del brazo. Borrar:

```xml
    <disable_collisions link1="baseFija_link" link2="manijaTrasera_link"          reason="Never"/>
    <disable_collisions link1="baseFija_link" link2="manijaDelantera_link"        reason="Never"/>
    <disable_collisions link1="caseBaseRobot_link" link2="manijaTrasera_link"          reason="Never"/>
    <disable_collisions link1="caseBaseRobot_link" link2="manijaDelantera_link"        reason="Never"/>
    <disable_collisions link1="zonaRecoleccion_link" link2="manijaTrasera_link"          reason="Never"/>
    <disable_collisions link1="zonaRecoleccion_link" link2="manijaDelantera_link"        reason="Never"/>
    <disable_collisions link1="airpump_link" link2="manijaTrasera_link"          reason="Never"/>
    <disable_collisions link1="airpump_link" link2="manijaDelantera_link"        reason="Never"/>
    <!-- manijaTrasera_link vs others -->
    <disable_collisions link1="manijaTrasera_link" link2="manijaDelantera_link"        reason="Never"/>
    <disable_collisions link1="manijaTrasera_link" link2="canecaLateralIzquierda_link" reason="Never"/>
    <disable_collisions link1="manijaTrasera_link" link2="canecaLateralDerecha_link"   reason="Never"/>
    <disable_collisions link1="manijaTrasera_link" link2="canecaFrontalDerecha_link"   reason="Never"/>
    <disable_collisions link1="manijaTrasera_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
    <!-- manijaDelantera_link vs others -->
    <disable_collisions link1="manijaDelantera_link" link2="canecaLateralIzquierda_link" reason="Never"/>
    <disable_collisions link1="manijaDelantera_link" link2="canecaLateralDerecha_link"   reason="Never"/>
    <disable_collisions link1="manijaDelantera_link" link2="canecaFrontalDerecha_link"   reason="Never"/>
    <disable_collisions link1="manijaDelantera_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
    <disable_collisions link1="${prefix}arm_base_link" link2="manijaTrasera_link"         reason="Never"/>
    <disable_collisions link1="${prefix}arm_base_link" link2="manijaDelantera_link"       reason="Never"/>
```

Y las mismas dos líneas del comentario de inventario (alrededor de la línea 578).

---

## Parte 2 — Añadir los links nuevos

Los cuatro links nuevos del kit son:
- `soporteCamara_link`
- `raspberry_link`
- `canastilla_link`
- `electronics_link`

### Dónde insertar

En ambos archivos, localiza el bloque que empieza con:
```xml
  <!-- canecaFrontalDerecha_link vs others -->
  <disable_collisions link1="canecaFrontalDerecha_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
```

Pega el bloque nuevo **inmediatamente después** de esa línea.

---

### Bloque para `phantomx_pincher.srdf`

```xml
  <!-- soporteCamara_link vs others -->
  <disable_collisions link1="soporteCamara_link" link2="baseFija_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="caseBaseRobot_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="zonaRecoleccion_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="airpump_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="canecaLateralIzquierda_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="canecaLateralDerecha_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="canecaFrontalDerecha_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="raspberry_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="canastilla_link" reason="Never"/>
  <disable_collisions link1="soporteCamara_link" link2="electronics_link" reason="Never"/>
  <!-- raspberry_link vs others -->
  <disable_collisions link1="raspberry_link" link2="baseFija_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="caseBaseRobot_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="zonaRecoleccion_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="airpump_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="canecaLateralIzquierda_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="canecaLateralDerecha_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="canecaFrontalDerecha_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="canastilla_link" reason="Never"/>
  <disable_collisions link1="raspberry_link" link2="electronics_link" reason="Never"/>
  <!-- canastilla_link vs others -->
  <disable_collisions link1="canastilla_link" link2="baseFija_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="caseBaseRobot_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="zonaRecoleccion_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="airpump_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="canecaLateralIzquierda_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="canecaLateralDerecha_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="canecaFrontalDerecha_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
  <disable_collisions link1="canastilla_link" link2="electronics_link" reason="Never"/>
  <!-- electronics_link vs others -->
  <disable_collisions link1="electronics_link" link2="baseFija_link" reason="Never"/>
  <disable_collisions link1="electronics_link" link2="caseBaseRobot_link" reason="Never"/>
  <disable_collisions link1="electronics_link" link2="zonaRecoleccion_link" reason="Never"/>
  <disable_collisions link1="electronics_link" link2="airpump_link" reason="Never"/>
  <disable_collisions link1="electronics_link" link2="canecaLateralIzquierda_link" reason="Never"/>
  <disable_collisions link1="electronics_link" link2="canecaLateralDerecha_link" reason="Never"/>
  <disable_collisions link1="electronics_link" link2="canecaFrontalDerecha_link" reason="Never"/>
  <disable_collisions link1="electronics_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
```

---

### Bloque para `phantomx_pincher.xacro`

Idéntico pero con `${prefix}` en el link del brazo:

```xml
    <!-- soporteCamara_link vs others -->
    <disable_collisions link1="soporteCamara_link" link2="baseFija_link"               reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="caseBaseRobot_link"          reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="zonaRecoleccion_link"        reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="airpump_link"                reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="canecaLateralIzquierda_link" reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="canecaLateralDerecha_link"   reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="canecaFrontalDerecha_link"   reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="raspberry_link"              reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="canastilla_link"             reason="Never"/>
    <disable_collisions link1="soporteCamara_link" link2="electronics_link"            reason="Never"/>
    <!-- raspberry_link vs others -->
    <disable_collisions link1="raspberry_link" link2="baseFija_link"               reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="caseBaseRobot_link"          reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="zonaRecoleccion_link"        reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="airpump_link"                reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="canecaLateralIzquierda_link" reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="canecaLateralDerecha_link"   reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="canecaFrontalDerecha_link"   reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="canastilla_link"             reason="Never"/>
    <disable_collisions link1="raspberry_link" link2="electronics_link"            reason="Never"/>
    <!-- canastilla_link vs others -->
    <disable_collisions link1="canastilla_link" link2="baseFija_link"               reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="caseBaseRobot_link"          reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="zonaRecoleccion_link"        reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="airpump_link"                reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="canecaLateralIzquierda_link" reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="canecaLateralDerecha_link"   reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="canecaFrontalDerecha_link"   reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
    <disable_collisions link1="canastilla_link" link2="electronics_link"            reason="Never"/>
    <!-- electronics_link vs others -->
    <disable_collisions link1="electronics_link" link2="baseFija_link"               reason="Never"/>
    <disable_collisions link1="electronics_link" link2="caseBaseRobot_link"          reason="Never"/>
    <disable_collisions link1="electronics_link" link2="zonaRecoleccion_link"        reason="Never"/>
    <disable_collisions link1="electronics_link" link2="airpump_link"                reason="Never"/>
    <disable_collisions link1="electronics_link" link2="canecaLateralIzquierda_link" reason="Never"/>
    <disable_collisions link1="electronics_link" link2="canecaLateralDerecha_link"   reason="Never"/>
    <disable_collisions link1="electronics_link" link2="canecaFrontalDerecha_link"   reason="Never"/>
    <disable_collisions link1="electronics_link" link2="canecaFrontalIzquierda_link" reason="Never"/>
```

---

## Resumen de cambios

| Acción | Link | Entradas afectadas |
|---|---|---|
| Eliminar | `manijaTrasera_link` | 10 líneas en cada archivo |
| Eliminar | `manijaDelantera_link` | 10 líneas en cada archivo |
| Añadir | `soporteCamara_link` | 11 líneas en cada archivo |
| Añadir | `raspberry_link` | 10 líneas en cada archivo |
| Añadir | `canastilla_link` | 9 líneas en cada archivo |
| Añadir | `electronics_link` | 8 líneas en cada archivo |

> **Nota:** el comentario de inventario al inicio de la sección del kit (alrededor de la línea 574
> en el `.xacro` y 380 en el `.srdf`) también debe actualizarse: eliminar `manijaTrasera_link` y
> `manijaDelantera_link`, y añadir `soporteCamara_link`, `raspberry_link`, `canastilla_link`,
> `electronics_link`.
