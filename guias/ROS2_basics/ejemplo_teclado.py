#!/usr/bin/env python3
import sys
# sys: módulo del sistema. Da acceso a sys.stdin (teclado), sys.stdout
# (pantalla), sys.argv (argumentos), sys.exit(), etc.

import termios
# termios: envoltorio de Python sobre el API POSIX (Linux/macOS, no Windows).
# Permite configurar el terminal a bajo nivel: eco, modo de entrada, manejo
# de Ctrl+C, etc.

import tty
# tty: funciones de alto nivel sobre termios. Su función estrella es
# tty.setraw(), que configura el modo crudo en una sola línea.

TECLAS = {
	'a' : 'te_amo', # up
	'b' : 'te_odio', # rest
	'c' : 'te_ignoro', # ready_near
	'd' : 'NO_IGNORES_QUE_TE_IGNORO' #ready_mid
}

def leer_teclado():
	fd = sys.stdin.fileno() # file descriptor
	# fileno(): devuelve el "descriptor de archivo" (un entero) de stdin,
	# normalmente 0. En Unix todo recurso de E/S se identifica con un número
	# (0=stdin, 1=stdout, 2=stderr). termios y tty no aceptan objetos Python,
	# solo este número.
	
	config_original = termios.tcgetattr(fd)
	# tcgetattr: "Terminal Control GET ATTRibutes". Devuelve la configuración
	# actual del terminal. La guardamos para restaurarla al final; si no, la
	# terminal queda en modo raw y se vuelve inusable hasta hacer 'reset'.
	
	print('Modo raw activo. Presiona a/b/c/d para probar, q para salir. \n')
	
	try:
		tty.setraw(fd)
		# setraw: activa modo raw. En modo normal (cooked) el terminal espera
		# Enter, hace eco, e interpreta Ctrl+C/Backspace. En raw todo eso se
		# desactiva: cada tecla llega al programa al instante, sin procesar.
		
		while True:
			tecla = sys.stdin.read(1)  # lee 1 carácter; en raw retorna sin esperar Enter
			
			if tecla == 'q':
				print('\r\nSaliendo.')
				# En raw, Ctrl+C no funciona como salida; por eso usamos 'q'.
				break
			
			if tecla in TECLAS:
				print(f'\r\nTecla: {tecla!r} significa que: {TECLAS[tecla]}')
				# \r y \n: caracteres herencia de máquinas de escribir.
				#   \n baja una línea pero NO regresa el cursor a la izquierda.
				#   \r regresa el cursor a la izquierda pero NO baja de línea.
				# En modo cooked Linux convierte \n en \r\n automáticamente;
				# en raw esa conversión se desactiva, por eso escribimos \r\n.
				#
				# !r aplica repr() a la variable dentro del f-string. repr()
				# muestra el carácter "como Python lo escribiría", con
				# comillas y secuencias visibles: 'a', '\x1b' (Escape), '\r'
				# (Enter). Sin !r, un carácter invisible no se vería.
				
			else:
				print(f'\r\nTecla desconocida: {tecla!r}')
		
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, config_original)
		# tcsetattr: "Terminal Control SET ATTRibutes". Restaura la config.
		# TCSADRAIN: indica CUÁNDO aplicar el cambio. Espera a que se envíe
		# todo el output pendiente antes de cambiar la configuración. Las
		# alternativas son TCSANOW (aplica de inmediato, puede cortar output
		# a medias) y TCSAFLUSH (descarta el input pendiente). TCSADRAIN es
		# el más seguro para restaurar.
		
		print('Terminal restaurada.')
		
if __name__ == '__main__':
	leer_teclado()
