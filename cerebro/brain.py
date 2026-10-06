"""Cerebro (PC) — monitor, registro, banco de pruebas y fase CEREBRO.

Durante un intento no toma ninguna decisión: eso vive en el firmware (reglamento 6.3).
  1. Lee la visión (TCP 2026) y muestra el mundo en el dashboard.
  2. Recibe el estado que publica cada rover (UDP BRAIN_STATUS_PORT).
  3. Graba cada ronda y vigila que los rovers reporten y avancen (recorder.py).
  4. Envía comandos de PRUEBA a los rovers (UDP ROVER_CMD_PORT) desde el dashboard.
  5. Fase CEREBRO, solo ANTES de READY (reglamento 6.2.5, 8.6.6): con los rovers quietos
     y los cubos en su lugar, calcula el reparto y las rutas (planner.py) y los carga en
     los rovers. Desde READY no transmite nada: todo envío pasa por transmit(), que se
     niega con la ronda en curso.

Los puntos 1 a 3 solo observan: pueden quedar encendidos durante un intento.

Uso:  vision-system/.venv/bin/python cerebro/brain.py   ->  http://localhost:8891
"""

import json
import os
import socket
import subprocess
import threading
import time

from flask import Flask, send_from_directory
from flask_socketio import SocketIO

import generador
import planner
from recorder import Recorder

# Las variables de entorno son solo para probar el cerebro sin cancha (visión y rovers simulados)
VISION_HOST = "127.0.0.1"
VISION_PORT = int(os.environ.get("CEREBRO_VISION_PORT", 2026))
ROVER_BCAST = os.environ.get("CEREBRO_ROVER_BCAST", "192.168.88.255")   # Broadcast de la red de los rovers
ROVER_CMD_PORT = int(os.environ.get("CEREBRO_CMD_PORT", 8889))          # Debe coincidir con ROVER_CMD_PORT del firmware
BRAIN_STATUS_PORT = int(os.environ.get("CEREBRO_STATUS_PORT", 8888))    # Debe coincidir con BRAIN_STATUS_PORT del firmware
WEB_PORT = int(os.environ.get("CEREBRO_WEB_PORT", 8891))
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


def recording_changed(on, name):
    """Se abrió o se cerró el archivo de una ronda."""
    socketio.emit("recording", recorder.info())
    if on:
        log("REG", f"Ronda en grabación: {name} ({recorder.info()['trigger']})")
    else:
        log("REG", f"Ronda guardada: {name}")
        socketio.emit("rounds", recorder.rounds())


recorder = Recorder(
    os.environ.get("CEREBRO_REG_DIR", os.path.join(HERE, "registros")),
    on_alert=lambda rover, what: log("ALERTA", f"Rover {rover}: {what}"),
    on_status=recording_changed,
)

# --- Ajustes de la cámara --------------------------------------------------------------
# La visión fija exposición y balance desde su configuración, pero NO la ganancia, que
# queda en cualquier valor cada vez que arranca; y de la ganancia depende que el cubo
# azul se vea con poca luz. Por eso el cerebro vuelve a aplicar el ajuste elegido
# (../camara.sh) cada vez que se conecta a la visión. Es la cámara de nuestra cancha de
# práctica: no tiene nada que ver con los rovers ni con la ronda.
CAMERA_SCRIPT = os.path.join(HERE, "..", "camara.sh")
CAMERA_FILE = os.path.join(HERE, "camara.txt")          # Último ajuste elegido: "dia" o "noche"


def camera_preset():
    try:
        with open(CAMERA_FILE) as f:
            return f.read().strip() or "noche"
    except OSError:
        return "noche"


def apply_camera(preset=None):
    preset = preset or camera_preset()
    if os.environ.get("CEREBRO_NO_CAMERA"):         # Prueba sin cancha: no tocar la cámara real
        return
    try:
        out = subprocess.run([CAMERA_SCRIPT, preset], capture_output=True, text=True, timeout=10)
        values = " · ".join(line.strip() for line in out.stdout.splitlines() if ":" in line)
        log("CAMARA", f"Ajuste '{preset}': {values or out.stderr.strip() or 'sin respuesta'}")
        socketio.emit("camera", {"preset": preset, "values": values})
    except (OSError, subprocess.SubprocessError) as e:
        log("CAMARA", f"No se pudo aplicar el ajuste '{preset}': {e}")


@socketio.on("camera")
def on_camera(data):
    """Elegir el ajuste de cámara desde el dashboard. data = {"preset": "dia" | "noche"}"""
    preset = "dia" if data.get("preset") == "dia" else "noche"
    with open(CAMERA_FILE, "w") as f:
        f.write(preset)
    apply_camera(preset)


# Disposición de práctica generada desde el dashboard (None = ninguna)
layout = None


def layout_message():
    if not layout:
        return None
    return {"nivel": layout["nivel"], "numero": layout["numero"], "metricas": layout["metricas"],
            "cubos": {c: list(p) for c, p in layout["cubos"].items()},
            "rovers": {str(r): list(p) for r, p in layout["rovers"].items()}}


@socketio.on("connect")
def on_connect():
    socketio.emit("layout", layout_message())
    socketio.emit("recording", recorder.info())
    socketio.emit("rounds", recorder.rounds())
    socketio.emit("camera", {"preset": camera_preset(), "values": ""})


@socketio.on("generate")
def on_generate(data):
    """Generar una disposición de práctica. data = {"nivel": 0.4, "numero": 7} o {"clear": true}.
    Solo se muestra en el mapa y se anota en la ronda: no se envía nada a los rovers."""
    global layout
    if data.get("clear"):
        layout = None
        log("NIVEL", "Sin disposición generada")
    else:
        layout = generador.generar(float(data.get("nivel", 0.4)), int(data.get("numero", 1)))
        log("NIVEL", f"Disposición nivel {layout['nivel']:.2f} n.º {layout['numero']}: distancia media "
                     f"{layout['metricas']['distancia']}, cruces {layout['metricas']['cruces']}")
    recorder.set_generated(layout)
    socketio.emit("layout", layout_message())


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


def transmit(target, cmd, quiet=False):
    """ÚNICA salida hacia los rovers. Con la ronda en curso no envía nada (reglamento
    6.3, 11.2.7). Devuelve True si salió."""
    if vision_phase in LOCKED_PHASES:
        if not quiet:
            log("BLOQUEADO", f"Ronda en curso ({vision_phase}): no se envía '{cmd[:40]}'. Reglamento 11.2.7.")
        return False
    payload = f"{target}:{cmd}"
    try:
        cmd_sock.sendto(payload.encode(), (ROVER_BCAST, ROVER_CMD_PORT))
    except OSError as e:                    # Sin la red de los rovers (cable, Wi-Fi): no es para caerse
        if not quiet:
            log("TX", f"No se pudo enviar '{cmd[:40]}': {e}")
        return False
    if not quiet:
        log("TX", payload if len(payload) < 90 else payload[:90] + "…")
    return True


@socketio.on("command")
def on_command(data):
    """data = {"rover": 10 | 11 | "*", "cmd": "r" | "f" | "M,0.4,0.4"}"""
    target = str(data.get("rover", "*"))
    cmd = str(data.get("cmd", "")).strip()
    if cmd:
        transmit(target, cmd)


# --- Fase CEREBRO ----------------------------------------------------------------------
# Antes de READY: cuando los rovers están quietos y los cubos dejaron de moverse, se
# calcula el plan (reparto, orden, cómo tomar cada cubo y por dónde ir) y se carga en
# cada rover, repitiendo el envío hasta que lo confirma en su estado. Si algo se mueve,
# se recalcula. Con la ronda en curso esta fase queda en pausa y no envía nada.
QUIET_S = 1.5               # Tiempo sin que nada se mueva antes de planificar
RESEND_S = 0.5
ROVER_ONLINE_S = 3.0
RESTING = ("IDLE", "FINISHED")          # Estados en que el rover acepta un plan
PRACTICE_WAIT_S = 6.0       # Lo máximo que "Iniciar práctica" espera a que el plan quede cargado

last_world = None
last_status = {}            # id de rover -> último estado (con "t" de llegada)
cerebro_on = True
cerebro = {"estado": "esperando", "texto": "", "plan": None, "ack": {}}
practice_until = 0.0        # > 0: hay un "Iniciar práctica" esperando al plan


def _moved(a, b, cube_tol, rover_tol):
    """¿Cambió el mundo entre 'a' y 'b' más que esas tolerancias (celdas)?"""
    ca = {c["color"]: c for c in a["cubes"]}
    cb = {c["color"]: c for c in b["cubes"]}
    ra = {r["id"]: r for r in a["rovers"]}
    rb = {r["id"]: r for r in b["rovers"]}
    if set(ca) != set(cb) or set(ra) != set(rb):
        return True
    if any(abs(ca[k]["col"] - cb[k]["col"]) > cube_tol or abs(ca[k]["row"] - cb[k]["row"]) > cube_tol for k in ca):
        return True
    return any(abs(ra[k]["col"] - rb[k]["col"]) > rover_tol or abs(ra[k]["row"] - rb[k]["row"]) > rover_tol or
               abs((ra[k]["theta"] - rb[k]["theta"] + 180) % 360 - 180) > 10 * rover_tol for k in ra)


def _cerebro_emit(estado, texto):
    changed = cerebro["estado"] != estado or cerebro["texto"] != texto
    cerebro["estado"], cerebro["texto"] = estado, texto
    plan = cerebro["plan"]
    socketio.emit("cerebro", {
        "activo": cerebro_on, "estado": estado, "texto": texto, "ack": cerebro["ack"],
        "plan": plan and {"id": plan["id"], "tareas": plan["tareas"], "comodin": plan["comodin"],
                          "juntos": plan["juntos"], "costo": plan["costo"], "dibujo": plan["dibujo"]}})
    if changed:
        log("CEREBRO", f"{estado}: {texto}" if texto else estado)


def _cerebro_run():
    global practice_until
    quiet_ref, quiet_since = None, 0.0
    plan_world, plan_rovers = None, ()
    last_send = 0.0
    while True:
        time.sleep(0.2)
        now = time.time()
        world = last_world
        online = {rid: s for rid, s in last_status.items() if now - s["t"] < ROVER_ONLINE_S}
        resting = sorted(rid for rid, s in online.items() if s.get("state") in RESTING)
        busy = [rid for rid in online if rid not in resting and online[rid].get("state") != "SIN_TELEMETRIA"]

        def start_practice(why):
            global practice_until
            practice_until = 0.0
            log("CEREBRO", f"Inicia la práctica ({why})")
            transmit("*", "S")

        if not cerebro_on:
            if practice_until:
                start_practice("fase CEREBRO apagada: planifican a bordo")
            _cerebro_emit("apagado", "los rovers planifican a bordo")
            continue
        if vision_phase in LOCKED_PHASES or busy:
            practice_until = 0.0
            _cerebro_emit("ronda", "en pausa: desde READY la PC no envía nada" if vision_phase in LOCKED_PHASES
                          else "en pausa: hay rovers trabajando")
            continue
        if world is None or not resting:
            _cerebro_emit("esperando", "sin visión" if world is None else "ningún rover en reposo reportando")
            continue
        seen = {r["id"] for r in world["rovers"]}
        rids = tuple(r for r in resting if r in seen)
        cubes = [c for c in world["cubes"] if c.get("age_ms", 0) < 1500]
        if not rids or not cubes:
            _cerebro_emit("esperando", "la cámara no ve a los rovers" if not rids else "sin cubos a la vista")
            continue
        view = dict(world, cubes=cubes)

        # ¿Quieto hace rato?
        if quiet_ref is None or _moved(view, quiet_ref, 0.6, 0.8):
            quiet_ref, quiet_since = view, now
        quiet = now - quiet_since >= QUIET_S

        plan = cerebro["plan"]
        stale = plan is None or rids != plan_rovers or _moved(view, plan_world, 1.0, 1.5)
        if stale and not quiet:
            _cerebro_emit("esperando", "algo se está moviendo")
            continue
        if stale:
            _cerebro_emit("calculando", f"rovers {', '.join(map(str, rids))}")
            t0 = time.time()
            try:
                plan = planner.plan_completo(view, int(now) % 30000 + 1, rovers=rids)
            except Exception as e:                      # Sin compilador, biblioteca rota...: se sigue sin plan
                plan = None
                log("CEREBRO", f"No se pudo planificar: {e}")
            plan_world, plan_rovers = view, rids
            cerebro["plan"], cerebro["ack"] = plan, {}
            recorder.set_plan(plan and {"id": plan["id"], "tareas": {str(k): v for k, v in plan["tareas"].items()},
                                        "comodin": plan["comodin"], "juntos": plan["juntos"], "costo": plan["costo"],
                                        "dibujo": {str(k): v for k, v in plan["dibujo"].items()}})
            if plan is None:
                if practice_until:
                    start_practice("sin plan de la PC: planifican a bordo")
                _cerebro_emit("sin plan", "no hay forma de repartir estos cubos; los rovers planifican a bordo")
                continue
            log("CEREBRO", f"Plan {plan['id']} en {1000 * (time.time() - t0):.0f} ms: " +
                " · ".join(f"R{rid} {plan['tareas'][rid] or '-'}" for rid in sorted(plan["tareas"])) +
                (f" · comodín {plan['comodin']}" if plan["comodin"] else "") +
                (" · salen a la vez" if plan["juntos"] else "") + f" · ~{plan['costo']:.0f} s")
            last_send = 0.0
        if plan is None:
            continue

        # Cargado cuando cada rover muestra este plan y la cantidad de rutas que se le mandó
        ack = {rid: online.get(rid, {}).get("plan") == plan["id"] and
               online.get(rid, {}).get("rt", -1) == len(plan["rutas"][rid]) for rid in plan["mensajes"]}
        cerebro["ack"] = {str(k): v for k, v in ack.items()}
        if all(ack.values()):
            if practice_until:
                start_practice("plan cargado")
            _cerebro_emit("cargado", f"plan {plan['id']}")
            continue
        if now - last_send >= RESEND_S:
            last_send = now
            for rid, ok in ack.items():
                if not ok:
                    transmit(rid, plan["mensajes"][rid], quiet=True)
        if practice_until and now > practice_until:
            start_practice("el plan no se confirmó a tiempo")
        _cerebro_emit("enviando", "falta confirmar: " + ", ".join(f"R{r}" for r, ok in ack.items() if not ok))


def cerebro_loop():
    """Mantiene viva la fase: un error se anota y la fase vuelve a empezar de cero."""
    while True:
        try:
            _cerebro_run()
        except Exception as e:
            log("CEREBRO", f"Error en la fase, se reinicia: {e!r}")
            cerebro["plan"], cerebro["ack"] = None, {}
            time.sleep(1.0)


@socketio.on("cerebro")
def on_cerebro(data):
    """Encender o apagar la fase CEREBRO desde el dashboard. data = {"on": true | false}"""
    global cerebro_on
    cerebro_on = bool(data.get("on"))
    if not cerebro_on:
        cerebro["plan"], cerebro["ack"] = None, {}
        recorder.set_plan(None)
    log("CEREBRO", "Encendida" if cerebro_on else "Apagada: los rovers planifican a bordo")


@socketio.on("practice")
def on_practice(_data=None):
    """Iniciar una práctica con los dos rovers: primero se asegura el plan cargado, después 'S'."""
    global practice_until
    now = time.time()
    working = [rid for rid, s in last_status.items()
               if now - s["t"] < ROVER_ONLINE_S and s.get("state") not in RESTING + ("SIN_TELEMETRIA",)]
    if working:
        transmit("*", "r")              # Venían de otra práctica: sin parar no aceptan un plan nuevo
    practice_until = now + PRACTICE_WAIT_S


def vision_loop():
    """Mantiene la conexión con la visión y emite el último mundo al dashboard."""
    global vision_phase, last_world
    while True:
        try:
            with socket.create_connection((VISION_HOST, VISION_PORT), timeout=3) as s:
                log("VISION", f"Conectado a {VISION_HOST}:{VISION_PORT}")
                # La visión acaba de arrancar (o de volver): reponer la ganancia y demás
                threading.Timer(3.0, apply_camera).start()
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
                    last_world = msg
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
        if "id" in status:
            last_status[status["id"]] = status
        recorder.rover(status)
        socketio.emit("rover_status", status)


def watchdog_loop():
    while True:
        time.sleep(1.0)
        recorder.watchdog()
        socketio.emit("recording", recorder.info())


if __name__ == "__main__":
    threading.Thread(target=vision_loop, daemon=True).start()
    threading.Thread(target=status_loop, daemon=True).start()
    threading.Thread(target=watchdog_loop, daemon=True).start()
    threading.Thread(target=cerebro_loop, daemon=True).start()
    print(f"[WEB] Dashboard en http://0.0.0.0:{WEB_PORT}")
    socketio.run(app, host="0.0.0.0", port=WEB_PORT, debug=False,
                 use_reloader=False, allow_unsafe_werkzeug=True)
