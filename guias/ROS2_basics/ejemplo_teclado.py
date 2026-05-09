#!/usr/bin/env python3
"""
Ejemplo de lectura de teclado en modo raw.

Script Python puro — sin ROS2.
Muestra cómo leer teclas una a una sin necesitar presionar Enter,
usando las librerías estándar tty, termios y sys.

Corre con:
    python3 ejemplo_teclado.py
"""

# sys permite acceder a la entrada estándar (teclado) a bajo nivel
import sys

# termios permite leer y modificar la configuración del terminal
# (velocidad, modo de entrada, eco de caracteres, etc.)
import termios

# tty ofrece funciones de alto nivel para cambiar el modo del terminal
# tty.setraw() es la que nos interesa: desactiva el procesamiento normal
# del terminal y entrega cada tecla al programa inmediatamente
import tty


# Teclas que el programa reconoce y su significado
TECLAS = {
    'a': 'arriba (up)',
    'b': 'descanso (rest)',
    'c': 'cerca (ready_near)',
    'd': 'medio (ready_mid)',
}


def leer_teclado():
    # fileno() devuelve el descriptor de archivo de stdin (normalmente es 0).
    # termios y tty trabajan con descriptores, no con objetos de archivo.
    fd = sys.stdin.fileno()

    # Guardamos la configuración actual del terminal ANTES de cambiarla.
    # Esto es fundamental: si el programa termina sin restaurarla, la terminal
    # queda en modo raw y deja de funcionar normalmente (no se verá lo que se escribe).
    config_original = termios.tcgetattr(fd)

    print('Modo raw activo. Presiona a/b/c/d para probar, q para salir.\n')

    try:
        # setraw() activa el modo raw en el terminal.
        # En modo normal (cooked), el terminal:
        #   - acumula caracteres hasta que el usuario presiona Enter
        #   - interpreta Ctrl+C, Ctrl+Z, Backspace, etc.
        #   - muestra en pantalla lo que se escribe (eco)
        # En modo raw, todo eso se desactiva: cada tecla llega al programa
        # de inmediato, sin eco y sin procesamiento especial.
        tty.setraw(fd)

        while True:
            # read(1) lee exactamente 1 carácter.
            # En modo raw, esto retorna en cuanto se presiona la tecla,
            # sin esperar Enter.
            tecla = sys.stdin.read(1)

            if tecla == 'q':
                # En modo raw Ctrl+C no interrumpe el programa, por eso
                # necesitamos una tecla de salida explícita como 'q'.
                print('\r\nSaliendo.')
                break

            if tecla in TECLAS:
                # \r es necesario en modo raw porque el terminal no convierte
                # automáticamente \n en \r\n, por lo que sin \r el cursor no
                # vuelve al inicio de la línea.
                print(f'\r\nTecla: {tecla!r} → {TECLAS[tecla]}')
            else:
                # repr() muestra caracteres especiales de forma legible.
                # Por ejemplo, la tecla Escape aparece como '\x1b', Enter como '\r'.
                print(f'\r\nTecla desconocida: {tecla!r}')

    finally:
        # El bloque finally se ejecuta SIEMPRE: tanto si el bucle termina
        # normalmente como si ocurre una excepción inesperada.
        # TCSADRAIN espera a que se envíe todo el output pendiente antes de
        # aplicar la configuración — más seguro que TCSANOW.
        termios.tcsetattr(fd, termios.TCSADRAIN, config_original)
        print('Terminal restaurada.')


if __name__ == '__main__':
    leer_teclado()
