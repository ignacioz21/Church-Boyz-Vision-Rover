import socket
import os

UDP_IP = "0.0.0.0"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

print(f"=== VISOR DE MAPA DEL ROVER ===")
print(f"Escuchando mapa en el puerto UDP {UDP_PORT}...")
print("Abre el Monitor Serie del ESP32 y presiona el boton BOOT o envia 'm' para ver el mapa aqui.")

# Ocultar cursor
print('\033[?25l', end="")

try:
    while True:
        data, addr = sock.recvfrom(4096)
        
        # Mover cursor arriba izquierda y limpiar pantalla suave (evita parpadeo)
        print('\033[H\033[2J', end="")
        
        # Imprimir el mapa con colores ANSI que envía el ESP32
        print(data.decode('utf-8', errors='ignore'))
except KeyboardInterrupt:
    print('\033[?25h') # Restaurar cursor
    print("\nSaliendo...")
