import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# En puntos de acceso (como iPhone Hotspot), el broadcast de subred (172.20.10.15)
# es más confiable que 255.255.255.255. También se puede pasar la IP del ESP32 por argumento.
destinos = ["255.255.255.255", "192.168.88.255"]
if len(sys.argv) > 1:
    destinos.insert(0, sys.argv[1])

print("=== CONTROL REMOTO INALÁMBRICO DEL ROVER ===")
print(f"Destinos UDP (puerto 8889): {destinos}")
print("Comandos disponibles:")
print("  p : Iniciar Patrullaje de bordes (Navegacion Telemetria)")
print("  3 : Cazar cubo ROJO y llevar al depósito")
print("  4 : Cazar cubo VERDE y llevar al depósito")
print("  5 : Cazar cubo AZUL y llevar al depósito")
print("  6 : Misión AUTONOMA (Cazar 2 cubos por cercanía)")
print("--- CEREBRO DIGITAL (central_brain.py) ---")
print("  t1 : Trazar ruta (Solo Rover 10)")
print("  e1 : Ejecutar ruta (Solo Rover 10)")
print("  t2 : Trazar ruta (Solo Rover 11)")
print("  e2 : Ejecutar ruta (Solo Rover 11)")
print("  tt : Trazar rutas (Ambos rovers)")
print("  ee : Ejecutar rutas (Ambos rovers - AUTO)")
print("  0  : IDLE (Detener todo)")
print("--- COMANDOS MANUALES ESP32 ---")
print("  r : STOP DE EMERGENCIA (Detener motores)")
print("  d : Alterar distancia de rotacion al cubo")
print("  f : Avanzar ambos motores 1.2 seg (Prueba)")
print("  i / l : Invertir motores derecho / izquierdo")
print("Escribe un comando y presiona Enter:")

while True:
    try:
        cmd = input("Comando > ")
        if cmd.strip():
            if cmd.lower().startswith('d'):
                try:
                    if ':' in cmd:
                        val = float(cmd.split(':')[1])
                    else:
                        val = float(input("Distancia en bloques (ej 8.5): "))
                    payload = f"D:{val}".encode()
                    for d in destinos:
                        try: sock.sendto(payload, (d, 8889))
                        except Exception: pass
                    print(f"-> Distancia {val} enviada.")
                    continue
                except:
                    print("Error formato.")
                    continue
                    
            cmd_str = cmd.strip().lower()
            brain_cmd = None
            if cmd_str == 't1': brain_cmd = b"B:PLAN_10"
            elif cmd_str == 'e1': brain_cmd = b"B:EXEC_10"
            elif cmd_str == 't2': brain_cmd = b"B:PLAN_11"
            elif cmd_str == 'e2': brain_cmd = b"B:EXEC_11"
            elif cmd_str == 'tt': brain_cmd = b"B:PLAN_ALL"
            elif cmd_str == 'ee': brain_cmd = b"B:AUTO"
            elif cmd_str == '0': brain_cmd = b"B:IDLE"
            
            if brain_cmd:
                try: sock.sendto(brain_cmd, ("127.0.0.1", 8890))
                except Exception: pass
                print(f"-> Comando {brain_cmd} enviado al Cerebro Digital.")
                continue

            # Comandos normales ESP32
            c = cmd.strip().encode() if ':' in cmd else cmd.strip()[0].encode()
            for d in destinos:
                try:
                    sock.sendto(c, (d, 8889))
                except Exception:
                    pass
    except (KeyboardInterrupt, EOFError):
        print("\nSaliendo...")
        break
    except Exception as e:
        print(f"Error: {e}")
