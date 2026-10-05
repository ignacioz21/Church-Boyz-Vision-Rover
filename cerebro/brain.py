"""Cerebro (PC) — monitor, registro y banco de pruebas de los rovers.

No toma decisiones de competencia: eso vive en el firmware (reglamento 6.3).
  1. Lee la visión (TCP 2026) y muestra el mundo en el dashboard.
  2. Recibe el estado que publica cada rover (UDP BRAIN_STATUS_PORT).
  3. Graba cada ronda y vigila que los rovers reporten y avancen (recorder.py).
  4. Envía comandos de PRUEBA a los rovers (UDP ROVER_CMD_PORT) desde el dashboard.

Los puntos 1 a 3 solo observan: pueden quedar encendidos durante un intento.

Uso:  vision-system/.venv/bin/python cerebro/brain.py   ->  http://localhost:8891
"""

import json
import os
import socket
import threading
import time

from flask import Flask, send_from_directory
from flask_socketio import SocketIO

from recorder import Recorder

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

# Última fase publicada por la visión. Desde READY el cerebro no transmite NADA a los
# rovers (reglamento 6.3, 11.2.7): solo escucha y graba.
vision_phase = None
LOCKED_PHASES = ("READY", "RUNNING")

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


def log(tag, msg):
    print(f"[{tag}] {msg}")
    socketio.emit("log", {"tag": tag, "msg": msg})


recorder = Recorder(
    os.path.join(HERE, "registros"),
    on_alert=lambda rover, what: log("ALERTA", f"Rover {rover}: {what}"),
    on_status=lambda on, name: socketio.emit("recording", {"on": on, "file": name}),
)


@socketio.on("record")
def on_record(data):
    """Grabar a mano (pruebas fuera de una ronda). data = {"on": true | false}"""
    recorder.set_forced(bool(data.get("on")))
    log("REG", f"Grabación manual {'iniciada: ' + recorder.name if data.get('on') else 'detenida'}")


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/rutas")
def routes_page():
    """Ventana aparte: el mapa en grande con la ruta que cada rover piensa recorrer."""
    return send_from_directory(app.static_folder, "rutas.html")


@socketio.on("command")
def on_command(data):
    """data = {"rover": 10 | 11 | "*", "cmd": "r" | "f" | "M,0.4,0.4"}"""
    target = str(data.get("rover", "*"))
    cmd = str(data.get("cmd", "")).strip()
    if not cmd:
        return
    if vision_phase in LOCKED_PHASES:
        log("BLOQUEADO", f"Ronda en curso ({vision_phase}): no se envía '{cmd}'. Reglamento 11.2.7.")
        return
    payload = f"{target}:{cmd}"
    cmd_sock.sendto(payload.encode(), (ROVER_BCAST, ROVER_CMD_PORT))
    log("TX", payload)


def vision_loop():
    """Mantiene la conexión con la visión y emite el último mundo al dashboard."""
    global vision_phase
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
                    vision_phase = msg.get("phase")
                    recorder.world(msg)
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
        data, addr = sock.recvfrom(2048)
        try:
            status = json.loads(data.decode())
        except ValueError:
            continue
        status["ip"] = addr[0]
        status["t"] = time.time()
        recorder.rover(status)
        socketio.emit("rover_status", status)


def watchdog_loop():
    while True:
        time.sleep(1.0)
        recorder.watchdog()


if __name__ == "__main__":
    threading.Thread(target=vision_loop, daemon=True).start()
    threading.Thread(target=status_loop, daemon=True).start()
    threading.Thread(target=watchdog_loop, daemon=True).start()
    print(f"[WEB] Dashboard en http://0.0.0.0:{WEB_PORT}")
    socketio.run(app, host="0.0.0.0", port=WEB_PORT, debug=False,
                 use_reloader=False, allow_unsafe_werkzeug=True)
