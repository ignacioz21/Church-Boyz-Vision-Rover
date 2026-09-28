import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

print("=== CONTROL REMOTO INALÁMBRICO DEL ROVER ===")
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
            # Envía el comando como broadcast a toda la red Wi-Fi
            sock.sendto(cmd.strip()[0].encode(), ("255.255.255.255", 8889))
    except KeyboardInterrupt:
        print("\nSaliendo...")
        break
    except Exception as e:
        print(f"Error: {e}")
