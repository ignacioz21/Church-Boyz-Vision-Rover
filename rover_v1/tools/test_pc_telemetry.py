#!/usr/bin/env python3
"""
test_pc_telemetry.py — Emulador y Verificador de Telemetría para el Rover V1
=============================================================================
Permite probar la recepción del rover levantando un servidor TCP local (puerto 2026)
que emite paquetes NDJSON oficiales, simulando las fases:
  IDLE (5 seg) -> READY (5 seg) -> RUNNING (30 seg)

Uso:
  python3 test_pc_telemetry.py --port 2026
"""

import socket
import json
import time
import argparse
import sys

def generar_frame(phase, step, qa_mode=False):
    # Coordenadas según modo
    if qa_mode:
        # Simulación de trayectoria de patrullaje de bordes (cuadrilátero 38x38)
        # Comienza en centro (20, 20) y avanza a la derecha (38, 20)
        if step < 40:
            col_rover = 20.0 + min(step * 0.45, 18.0)
            row_rover = 20.0
            heading_rover = 0.0
        elif step < 60: # Giro 90° a la izquierda (UP)
            col_rover = 38.0
            row_rover = 20.0
            heading_rover = min((step - 40) * 4.5, 90.0)
        elif step < 120: # Avanza hacia arriba hasta (38, 5)
            col_rover = 38.0
            row_rover = 20.0 - min((step - 60) * 0.25, 15.0)
            heading_rover = 90.0
        elif step < 140: # Giro 90° a la izquierda (LEFT)
            col_rover = 38.0
            row_rover = 5.0
            heading_rover = 90.0 + min((step - 120) * 4.5, 90.0)
        elif step < 220: # Avanza a la izquierda hasta (5, 5)
            col_rover = 38.0 - min((step - 140) * 0.41, 33.0)
            row_rover = 5.0
            heading_rover = 180.0
        elif step < 240: # Giro 90° a la izquierda (DOWN)
            col_rover = 5.0
            row_rover = 5.0
            heading_rover = 180.0 + min((step - 220) * 4.5, 90.0)
        elif step < 320: # Avanza hacia abajo hasta (5, 38)
            col_rover = 5.0
            row_rover = 5.0 + min((step - 240) * 0.41, 33.0)
            heading_rover = 270.0
        elif step < 340: # Giro 90° a la izquierda (RIGHT)
            col_rover = 5.0
            row_rover = 38.0
            heading_rover = 270.0 + min((step - 320) * 4.5, 90.0)
        else: # Avanza a la derecha hasta (38, 38)
            col_rover = 5.0 + min((step - 340) * 0.41, 33.0)
            row_rover = 38.0
            heading_rover = 0.0
    else:
        # Rover ID 10 en posición inicial
        col_rover = 10.0 + min(step * 0.2, 18.0)
        row_rover = 20.0
        heading_rover = 0.0

    # Cubo verde en (30.0, 20.0)
    col_green = 30.0
    row_green = 20.0
    if phase == "RUNNING" and not qa_mode and col_rover > 25.0:
        col_green = min(col_rover + 2.0, 39.0)

    frame = {
        "v": 2,
        "phase": phase,
        "clock": "01:30",
        "grid": {
            "cols": 43,
            "rows": 43,
            "cell_mm": 20.0
        },
        "rovers": [
            {
                "id": 10,
                "col": round(col_rover, 3),
                "row": round(row_rover, 3),
                "theta": round(heading_rover, 1),
                "heading": round(heading_rover, 1),
                "age_ms": 0
            }
        ],
        "cubes": [
            {
                "color": "green",
                "col": round(col_green, 3),
                "row": round(row_green, 3),
                "age_ms": 0
            },
            {
                "color": "red",
                "col": 33.0,
                "row": 10.0,
                "age_ms": 0
            },
            {
                "color": "blue",
                "col": 15.0,
                "row": 35.0,
                "age_ms": 0
            }
        ],
        "depots": [
            {
                "color": "green",
                "col": 39.25,
                "row": 21.5
            },
            {
                "color": "red",
                "col": 25.0,
                "row": 4.0
            },
            {
                "color": "blue",
                "col": 25.0,
                "row": 36.0
            }
        ]
    }
    return json.dumps(frame) + "\n"

def main():
    parser = argparse.ArgumentParser(description="Emulador de telemetría TCP para Rover V1 y QA")
    parser.add_argument("--host", default="0.0.0.0", help="IP de escucha (0.0.0.0 para toda la red)")
    parser.add_argument("--port", type=int, default=2026, help="Puerto TCP (por defecto 2026)")
    parser.add_argument("--qa-perimeter", action="store_true", help="Simula trayectoria de patrullaje de bordes para prueba QA")
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((args.host, args.port))
    except Exception as e:
        print(f"[ERROR] No se pudo enlazar al puerto {args.port}: {e}")
        sys.exit(1)

    server.listen(2)
    print(f"============================================================")
    print(f"  Servidor de Telemetría Simulado activo en {args.host}:{args.port}")
    if args.qa_perimeter:
        print(f"  [MODO ACTIVO]: Simulación de Patrullaje de Bordes QA")
    print(f"  Esperando conexión del rover ESP32...")
    print(f"============================================================")

    while True:
        client, addr = server.accept()
        print(f"[CONEXION] Rover conectado desde {addr[0]}:{addr[1]}")

        def reader():
            buf = ""
            while True:
                try:
                    data = client.recv(512)
                    if not data:
                        break
                    buf += data.decode("utf-8", errors="ignore")
                    while "\n" in buf:
                        l, buf = buf.split("\n", 1)
                        l = l.strip()
                        if l:
                            print(f"\n{l}")
                except Exception:
                    break

        import threading
        t = threading.Thread(target=reader, daemon=True)
        t.start()

        start_time = time.time()
        step = 0

        try:
            while True:
                elapsed = time.time() - start_time
                if elapsed < 5.0:
                    phase = "IDLE"
                elif elapsed < 10.0:
                    phase = "READY"
                elif elapsed < 60.0:
                    phase = "RUNNING"
                    step += 1
                else:
                    phase = "FINISHED"

                line = generar_frame(phase, step, qa_mode=args.qa_perimeter)
                client.sendall(line.encode("utf-8"))

                print(f"\r[ENVIO 20Hz] Fase: {phase:<8} | t={elapsed:5.1f}s | Step: {step}", end="")
                time.sleep(0.05) # 20 Hz
        except (BrokenPipeError, ConnectionResetError):
            print(f"\n[DESCONEXION] Rover desconectado. Esperando nueva conexión...")
            client.close()

if __name__ == "__main__":
    main()
