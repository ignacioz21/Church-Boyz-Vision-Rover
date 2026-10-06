"""Registro y vigilante de rondas.

Graba SIEMPRE lo que pasa (mundo, estado de cada rover) y, además, arma un archivo por
ronda que abre y cierra solo:
  - empieza cuando la visión pasa a READY, cuando un rover empieza a trabajar
    (práctica) o cuando se pulsa Grabar;
  - termina cuando los tres cubos quedan en zona, cuando ningún rover sigue trabajando,
    cuando la visión termina la ronda, o a los 10 minutos.
Cada ronda lleva una ficha con la disposición de cubos, el nivel del generador al que
corresponde y con qué firmware se hizo, y al cerrar se anota en un índice.

Solo OBSERVA: no le manda nada a los rovers, así que puede quedar encendido durante
un intento (reglamento 6.3).

Archivos (cerebro/registros/):
    continuo/AAAAMMDD_HH.jsonl      todo, siempre (se borra solo a los 3 días)
    ronda_AAAAMMDD_HHMMSS.jsonl     una ronda; una línea JSON por dato:
        {"t": 0.0,  "type": "ficha", ...}    disposición, nivel, firmware, cómo empezó
        {"t": 12.3, "type": "world", ...}    el mundo según la visión (5 por segundo)
        {"t": 12.4, "type": "rover", ...}    lo que reporta un rover
        {"t": 15.0, "type": "event", ...}    un cubo entró o salió de su zona, cambio de fase
        {"t": 20.1, "type": "alert", ...}    aviso del vigilante
        {"t": 61.0, "type": "summary", ...}  resultado al cerrar
    indice.csv                      una fila por ronda
"""

import collections
import csv
import glob
import json
import os
import threading
import time

import nivel
import planner

WORLD_PERIOD_S = 0.2        # Cada cuánto se graba el mundo
OFFLINE_S = 1.5             # Sin reportes de un rover
STALE_S = 2.0               # Rover sin telemetría fresca
STALL_S = 15.0              # Rover que debería moverse y no avanza
STALL_CELLS = 1.5
# Estados en los que es normal estar quieto
RESTING = {"IDLE", "READY", "RUNNING", "FINISHED", "ESPERA", "FIN", "ESPERAR", "CEDER", "VERIFICAR",
           "SIN_TELEMETRIA", "PRUEBA"}

# Detección de ronda
WORKING = {"PLANIFICANDO", "IR_A_PREPARAR", "APUNTAR", "CAPTURAR", "REAPUNTAR", "TRANSPORTAR", "ENTREGAR",
           "HACER_LUGAR", "APARTAR", "SOLTAR", "VERIFICAR", "DESPEJAR", "CEDER", "OBSTACULO"}
PREROLL_S = 5.0             # Lo anterior al inicio que se incluye en el archivo de la ronda
DONE_HOLD_S = 2.0           # Los tres cubos en zona durante este tiempo => terminó
IDLE_HOLD_S = 6.0           # Ningún rover trabajando durante este tiempo => terminó
ROUND_MAX_S = 600.0         # Duración máxima de un intento (reglamento 10.1)
QUIET_S = 1.0               # Cubos quietos este tiempo => esa es la disposición
QUIET_CELLS = 0.4
MISPLACED_CELLS = 2.0       # Cubo a más de esto de su sombra => ronda "mal colocada"
CONTINUOUS_KEEP_DAYS = 3

INDEX_FIELDS = ["ronda", "inicio", "disparador", "nivel", "numero", "nivel_estimado", "grupo", "encaje",
                "del_generador", "mal_colocada", "distancia", "cruces", "clave", "rovers", "firmware", "plan",
                "duracion", "entregados", "t_tercer_cubo", "t_red", "t_green", "t_blue",
                "incidentes", "obstaculos", "reinicios", "cierre"]


class Recorder:
    def __init__(self, directory, on_alert=None, on_status=None):
        self.directory = directory
        self.on_alert = on_alert or (lambda rover, what: None)
        self.on_status = on_status or (lambda recording, name: None)
        self.file = None                # Archivo de la ronda en curso
        self.name = None
        self.t0 = 0.0
        self.forced = False             # Grabación pedida a mano: no se cierra sola
        self.trigger = None
        self.phase = "IDLE"
        self.last_world_s = 0.0
        self.in_depot = {}              # color -> t en que entró a su zona
        self.rovers = {}                # id -> {seen, moved_at, pos, alerts, state, status}
        self.counts = {}
        self.card = {}                  # Ficha de la ronda en curso
        self.all_done_since = None
        self.all_idle_since = None
        self.resets = 0
        self.generated = None           # Disposición generada desde el dashboard (generador.generar)
        self.brain_plan = None          # Plan que cargó la fase CEREBRO (reparto y rutas), para comparar con lo hecho
        self.preroll = collections.deque()          # (t absoluto, tipo, datos) de los últimos segundos
        self.cube_track = collections.deque()       # (t, {color: (col, row)}) para saber cuándo están quietos
        self.quiet_layout = None        # Última disposición con los cubos quietos
        self.pending_cubes = 0          # Cubos a la vista que todavía no están en su zona
        self.cont_file = None
        self.cont_hour = None
        # La visión, los rovers y el vigilante llegan por hilos distintos
        self.lock = threading.RLock()

    # --- Grabación continua -----------------------------------------------------
    def _continuous(self, now, kind, data):
        hour = time.strftime("%Y%m%d_%H", time.localtime(now))
        if hour != self.cont_hour:
            if self.cont_file:
                self.cont_file.close()
            folder = os.path.join(self.directory, "continuo")
            os.makedirs(folder, exist_ok=True)
            self.cont_file = open(os.path.join(folder, hour + ".jsonl"), "a")
            self.cont_hour = hour
            limit = now - CONTINUOUS_KEEP_DAYS * 86400
            for old in glob.glob(os.path.join(folder, "*.jsonl")):
                if os.path.getmtime(old) < limit:
                    os.remove(old)
        self.cont_file.write(json.dumps({"ts": round(now, 2), "type": kind, **data}) + "\n")
        self.cont_file.flush()

    def _record(self, kind, **data):
        """Un dato: va a la grabación continua, a la memoria previa y, si hay ronda, a su archivo."""
        now = time.time()
        self._continuous(now, kind, data)
        self.preroll.append((now, kind, data))
        while self.preroll and now - self.preroll[0][0] > PREROLL_S:
            self.preroll.popleft()
        self._write(kind, now=now, **data)

    # --- Archivo de la ronda -----------------------------------------------------
    def _write(self, kind, now=None, **data):
        if not self.file:
            return
        self.counts[kind] = self.counts.get(kind, 0) + 1
        t = (now if now is not None else time.time()) - self.t0
        self.file.write(json.dumps({"t": round(t, 2), "type": kind, **data}) + "\n")
        self.file.flush()

    def _layout_now(self):
        return self.cube_track[-1][1] if self.cube_track else {}

    def _open(self, why):
        os.makedirs(self.directory, exist_ok=True)
        now = time.time()
        self.name = time.strftime("ronda_%Y%m%d_%H%M%S.jsonl", time.localtime(now))
        self.file = open(os.path.join(self.directory, self.name), "w")
        self.t0 = now
        self.trigger = why
        self.in_depot = {}
        self.counts = {"world": 0, "rover": 0, "event": 0, "alert": 0}
        self.all_done_since = self.all_idle_since = None
        self.resets = 0
        for r in self.rovers.values():
            r["alerts"] = {}
            r["moved_at"] = now

        # Ficha: la disposición es la última con los cubos quietos (no el cuadro del
        # arranque, que puede traer un cubo ya movido)
        layout = self.quiet_layout or self._layout_now()
        cubes = {c: [round(p[0], 1), round(p[1], 1)] for c, p in layout.items()}
        info = nivel.clasificar({c: tuple(p) for c, p in cubes.items()}) or {}
        card = {
            "disparador": why, "inicio": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
            "cubos": cubes,
            "rovers": {str(rid): [round(r["pos"][0], 1), round(r["pos"][1], 1)]
                       for rid, r in self.rovers.items() if r.get("pos") and now - r.get("seen", 0) < OFFLINE_S},
            "firmware": {str(rid): r["status"].get("fw", "?") for rid, r in self.rovers.items() if r.get("status")},
            "plan": {str(rid): r["status"].get("plan", 0) for rid, r in self.rovers.items() if r.get("status")},
            "nivel_estimado": info.get("nivel"), "grupo": info.get("grupo"), "encaje": info.get("encaje"),
            "del_generador": info.get("del_generador"), "distancia": info.get("distancia"),
            "cruces": info.get("cruces"), "clave": info.get("clave"),
            "nivel": None, "numero": None, "mal_colocada": None,
        }
        if self.brain_plan:
            card["cerebro"] = self.brain_plan
        if self.generated:
            off = nivel.desvio({c: tuple(p) for c, p in cubes.items()}, self.generated)
            card.update({"nivel": self.generated["nivel"], "numero": self.generated["numero"],
                         "sombras": {c: list(p) for c, p in self.generated["cubos"].items()},
                         "desvio": off,
                         "mal_colocada": len(off) < 3 or max(off.values()) > MISPLACED_CELLS})
            card["grupo"] = min(nivel.GRUPOS, key=lambda g: abs(g - self.generated["nivel"]))
        self.card = card
        self._write("ficha", now=now, **card)
        # Lo que pasó justo antes del inicio, con su tiempo real (negativo)
        for t_abs, kind, data in list(self.preroll):
            if t_abs < now:
                self._write(kind, now=t_abs, **data)
        self._write("event", now=now, what="inicio", why=why)
        self.on_status(True, self.name)

    def _close(self, why):
        if not self.file:
            return
        now = time.time()
        times = {c: t for c, t in self.in_depot.items()}
        incidents = sum(r["status"].get("st", [0] * 5)[4] for r in self.rovers.values() if r.get("status"))
        obstacles = sum(r["status"].get("ob", 0) for r in self.rovers.values() if r.get("status"))
        result = {
            "why": why, "duration": round(now - self.t0, 1), "delivered": times,
            "t_third": round(max(times.values()), 1) if len(times) >= 3 else None,
            "incidents": incidents, "obstacles": obstacles, "resets": self.resets, "counts": self.counts,
        }
        self._write("summary", now=now, **result)
        self.file.close()
        self.file = None
        self._index(result)
        self.on_status(False, self.name)

    def _index(self, result):
        path = os.path.join(self.directory, "indice.csv")
        card = self.card
        row = {
            "ronda": self.name[6:-6], "inicio": card.get("inicio"), "disparador": card.get("disparador"),
            "nivel": card.get("nivel"), "numero": card.get("numero"), "nivel_estimado": card.get("nivel_estimado"),
            "grupo": card.get("grupo"), "encaje": card.get("encaje"), "del_generador": card.get("del_generador"),
            "mal_colocada": card.get("mal_colocada"), "distancia": card.get("distancia"), "cruces": card.get("cruces"),
            "clave": card.get("clave"), "rovers": " ".join(sorted(card.get("rovers", {}))),
            "firmware": " ".join(f"{k}:{v}" for k, v in sorted(card.get("firmware", {}).items())),
            "plan": " ".join(f"{k}:{v}" for k, v in sorted(card.get("plan", {}).items())),
            "duracion": result["duration"], "entregados": len(result["delivered"]),
            "t_tercer_cubo": result["t_third"], "t_red": result["delivered"].get("red"),
            "t_green": result["delivered"].get("green"), "t_blue": result["delivered"].get("blue"),
            "incidentes": result["incidents"], "obstaculos": result["obstacles"], "reinicios": result["resets"],
            "cierre": result["why"],
        }
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=INDEX_FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)

    # --- Órdenes del dashboard ----------------------------------------------------
    def set_forced(self, on):
        with self.lock:
            return self._set_forced(on)

    def _set_forced(self, on):
        """Grabar a mano: se abre ya y no se cierra hasta que se pida."""
        self.forced = on
        if on and not self.file:
            self._open("manual")
        elif not on and self.file:
            self._close("manual")

    def set_generated(self, layout):
        with self.lock:
            return self._set_generated(layout)

    def _set_generated(self, layout):
        """Disposición generada en el dashboard (o None): es el nivel de las rondas que sigan."""
        self.generated = layout

    def set_plan(self, plan):
        """Plan de la fase CEREBRO vigente (o None): queda anotado en la ficha de la próxima ronda."""
        self.brain_plan = plan

    def info(self):
        """Estado para el dashboard."""
        return {"on": self.file is not None, "file": self.name, "trigger": self.trigger if self.file else None,
                "elapsed": round(time.time() - self.t0, 1) if self.file else None,
                "nivel": self.card.get("nivel") if self.file else None,
                "numero": self.card.get("numero") if self.file else None,
                "nivel_estimado": self.card.get("nivel_estimado") if self.file else None}

    def rounds(self, last=12):
        """Las últimas rondas del índice, de la más nueva a la más vieja."""
        path = os.path.join(self.directory, "indice.csv")
        if not os.path.exists(path):
            return []
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
        return rows[-last:][::-1]

    # --- Entradas -----------------------------------------------------------------
    def world(self, msg):
        with self.lock:
            return self._world(msg)

    def _world(self, msg):
        now = time.time()
        phase = msg.get("phase", "IDLE")

        # Seguimiento de los cubos: la disposición "quieta" más reciente
        cubes = {c["color"]: (c["col"], c["row"]) for c in msg.get("cubes", []) if c.get("age_ms", 0) < 500}
        self.cube_track.append((now, cubes))
        while self.cube_track and now - self.cube_track[0][0] > QUIET_S + 0.5:
            self.cube_track.popleft()
        old = self.cube_track[0]
        if not self.file and now - old[0] >= QUIET_S and cubes and set(cubes) == set(old[1]) and \
                all(abs(cubes[c][0] - old[1][c][0]) + abs(cubes[c][1] - old[1][c][1]) < QUIET_CELLS for c in cubes):
            self.quiet_layout = cubes

        depots_now = {d["color"]: d for d in msg.get("depots", [])}
        self.pending_cubes = sum(
            1 for c in msg.get("cubes", [])
            if c["color"] in depots_now and not planner.cube_in_depot(c, depots_now[c["color"]], msg["depot_size"],
                                                                     msg["grid"], msg["cube_side"]))

        if phase != self.phase:
            # Ronda oficial: de READY hasta que la visión la termina
            if phase in ("READY", "RUNNING") and not self.file:
                self._open("oficial")
            self._record("event", what="fase", phase=phase)
            if phase in ("FINISHED", "IDLE") and self.file and self.trigger == "oficial" and not self.forced:
                self.phase = phase
                self._close("fin de ronda")
            self.phase = phase

        if now - self.last_world_s >= WORLD_PERIOD_S:
            self.last_world_s = now
            self._record("world", phase=phase, clock=msg.get("clock"), seq=msg.get("seq"),
                         rovers=msg.get("rovers", []), cubes=msg.get("cubes", []))
        if not self.file:
            return

        # Cubos que entran o salen de su zona
        depots = {d["color"]: d for d in msg.get("depots", [])}
        for cube in msg.get("cubes", []):
            color = cube["color"]
            if color not in depots:
                continue
            inside = planner.cube_in_depot(cube, depots[color], msg["depot_size"], msg["grid"], msg["cube_side"])
            if inside and color not in self.in_depot:
                self.in_depot[color] = round(now - self.t0, 1)
                self._record("event", what="cubo_en_zona", color=color)
            elif not inside and color in self.in_depot and cube.get("age_ms", 0) < 300:
                del self.in_depot[color]
                self._record("event", what="cubo_fuera_de_zona", color=color)

        # Fin: los tres cubos en zona durante un momento
        if len(self.in_depot) >= 3:
            self.all_done_since = self.all_done_since or now
            if now - self.all_done_since > DONE_HOLD_S and not self.forced:
                self._close("tres cubos en zona")
                return
        else:
            self.all_done_since = None
        if now - self.t0 > ROUND_MAX_S and not self.forced:
            self._close("10 minutos")

    def rover(self, status):
        with self.lock:
            return self._rover(status)

    def _rover(self, status):
        now = time.time()
        rid = status.get("id")
        r = self.rovers.setdefault(rid, {"alerts": {}, "moved_at": now, "pos": None})
        pos = (status.get("col", 0.0), status.get("row", 0.0))
        if r["pos"] is None or abs(pos[0] - r["pos"][0]) + abs(pos[1] - r["pos"][1]) > STALL_CELLS:
            r["pos"], r["moved_at"] = pos, now
        if status.get("state") in RESTING:
            r["moved_at"] = now
        previous = r.get("status")
        if previous and status.get("up", 1e9) < previous.get("up", 0) and self.file:
            self.resets += 1
            self._record("event", what="reinicio", rover=rid, rst=status.get("rst"), crumb=status.get("crumb"))
        was_working = r.get("state") in WORKING
        r["seen"] = now
        r["fresh"] = status.get("fresh", False)
        r["stale_since"] = None if r["fresh"] else (r.get("stale_since") or now)
        r["state"] = status.get("state")
        r["status"] = status

        # Práctica: un rover que empieza a trabajar abre la ronda
        # (si ya no queda ningún cubo por entregar, no es una ronda nueva)
        if not self.file and r["state"] in WORKING and not was_working and self.pending_cubes > 0:
            self._open("practica")
        self._record("rover", **{k: v for k, v in status.items() if k not in ("t", "ip")})

    # --- Vigilante: llamar una vez por segundo --------------------------------------
    def watchdog(self):
        with self.lock:
            return self._watchdog()

    def _watchdog(self):
        now = time.time()
        for rid, r in self.rovers.items():
            checks = {
                "sin reportes": now - r.get("seen", now) > OFFLINE_S,
                "sin telemetría": r.get("stale_since") is not None and now - r["stale_since"] > STALE_S,
                "sin avanzar (%s)" % r.get("state"): now - r["moved_at"] > STALL_S and now - r.get("seen", 0) < OFFLINE_S,
            }
            for what, bad in checks.items():
                key = what.split(" (")[0]
                if bad and not r["alerts"].get(key):
                    r["alerts"][key] = True
                    self._record("alert", rover=rid, what=what)
                    self.on_alert(rid, what)
                elif not bad and r["alerts"].get(key):
                    r["alerts"][key] = False
                    self._record("alert", rover=rid, what=key + ": resuelto")
                    self.on_alert(rid, key + ": resuelto")

        # Fin de una práctica: ningún rover sigue trabajando
        if self.file and self.trigger == "practica" and not self.forced:
            busy = any(r.get("state") in WORKING and now - r.get("seen", 0) < OFFLINE_S for r in self.rovers.values())
            if busy:
                self.all_idle_since = None
            else:
                self.all_idle_since = self.all_idle_since or now
                if now - self.all_idle_since > IDLE_HOLD_S:
                    self._close("ningún rover trabajando")
