import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# En puntos de acceso (como iPhone Hotspot), el broadcast de subred (172.20.10.15)
# es más confiable que 255.255.255.255. También se puede pasar la IP del ESP32 por argumento.
destinos = ["255.255.255.255", "172.20.10.15"]
if len(sys.argv) > 1:
    destinos.insert(0, sys.argv[1])

print("=== CONTROL REMOTO INALÁMBRICO DEL ROVER ===")
print(f"Destinos UDP (puerto 8889): {destinos}")
print("Comandos disponibles:")
print("  p : Iniciar Patrullaje de bordes (Navegacion Telemetria)")
print("  3 : Cazar cubo ROJO y llevar al depósito")
print("  4 : Cazar cubo VERDE y llevar al depósito")
print("  5 : Cazar cubo AZUL y llevar al depósito")
print("  r : STOP DE EMERGENCIA (Detener motores)")
print("  f : Avanzar ambos motores 1.2 seg (Prueba)")
print("  i / l : Invertir motores derecho / izquierdo")
print("Escribe un comando y presiona Enter:")

while True:
    try:
        cmd = input("Comando > ")
        if cmd.strip():
            c = cmd.strip()[0].encode()
            for d in destinos:
                try:
                    sock.sendto(c, (d, 8889))
                except Exception:
                    pass
    except KeyboardInterrupt:
        print("\nSaliendo...")
        break
    except Exception as e:
        print(f"Error: {e}")
