"""Cerebro (PC) — monitor y banco de pruebas de los rovers.

No toma decisiones de competencia: eso vive en el firmware (reglamento 6.3).
Hace tres cosas:
  1. Lee la visión (TCP 2026) y muestra el mundo en el dashboard.
  2. Recibe el estado que publica cada rover (UDP BRAIN_STATUS_PORT).
  3. Envía comandos de PRUEBA a los rovers (UDP ROVER_CMD_PORT) desde el dashboard.

Uso:  vision-system/.venv/bin/python cerebro/brain.py   ->  http://localhost:8891
"""

import json
import os
import socket
import threading
import time

from flask import Flask, send_from_directory
from flask_socketio import SocketIO

VISION_HOST = "127.0.0.1"
VISION_PORT = 2026
ROVER_BCAST = "192.168.88.255"   # Broadcast de la red de los rovers
ROVER_CMD_PORT = 8889            # Debe coincidir con ROVER_CMD_PORT del firmware
BRAIN_STATUS_PORT = 8888         # Debe coincidir con BRAIN_STATUS_PORT del firmware
WEB_PORT = 8891
WORLD_EMIT_HZ = 10

HERE = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, static_folder=os.path.join(HERE, "dashboard"))
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


def log(tag, msg):
    print(f"[{tag}] {msg}")
    socketio.emit("log", {"tag": tag, "msg": msg})


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@socketio.on("command")
def on_command(data):
    """data = {"rover": 10 | 11 | "*", "cmd": "r" | "f" | "M,0.4,0.4"}"""
    target = str(data.get("rover", "*"))
    cmd = str(data.get("cmd", "")).strip()
    if not cmd:
        return
    payload = f"{target}:{cmd}"
    cmd_sock.sendto(payload.encode(), (ROVER_BCAST, ROVER_CMD_PORT))
    log("TX", payload)


def vision_loop():
    """Mantiene la conexión con la visión y emite el último mundo al dashboard."""
    while True:
        try:
            with socket.create_connection((VISION_HOST, VISION_PORT), timeout=3) as s:
                log("VISION", f"Conectado a {VISION_HOST}:{VISION_PORT}")
                buffer = b""
                last_emit = 0.0
                while True:
                    chunk = s.recv(65536)
                    if not chunk:
                        raise ConnectionError("la visión cerró la conexión")
                    buffer += chunk
                    lines = buffer.split(b"\n")
                    buffer = lines.pop()             # Resto incompleto
                    if not lines or time.time() - last_emit < 1.0 / WORLD_EMIT_HZ:
                        continue
                    msg = json.loads(lines[-1])       # Solo el más reciente
                    if msg.get("v") != 2:
                        continue
                    last_emit = time.time()
                    socketio.emit("world", msg)
        except (OSError, ConnectionError, ValueError) as e:
            log("VISION", f"Sin visión ({e}). Reintentando...")
            time.sleep(2)


def status_loop():
    """Recibe el estado JSON que publica cada rover y lo reenvía al dashboard."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", BRAIN_STATUS_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        try:
            status = json.loads(data.decode())
        except ValueError:
            continue
        status["ip"] = addr[0]
        status["t"] = time.time()
        socketio.emit("rover_status", status)


if __name__ == "__main__":
    threading.Thread(target=vision_loop, daemon=True).start()
    threading.Thread(target=status_loop, daemon=True).start()
    print(f"[WEB] Dashboard en http://0.0.0.0:{WEB_PORT}")
    socketio.run(app, host="0.0.0.0", port=WEB_PORT, debug=False,
                 use_reloader=False, allow_unsafe_werkzeug=True)
