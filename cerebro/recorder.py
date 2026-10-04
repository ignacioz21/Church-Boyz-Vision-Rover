"""Registro y vigilante de rondas.

Graba lo que pasa (mundo, estado de cada rover, eventos) en un archivo por ronda,
para analizarlo después, y avisa cuando algo va mal: un rover que no reporta, que
perdió la telemetría o que no avanza.

Solo OBSERVA: no le manda nada a los rovers, así que puede quedar encendido durante
un intento (reglamento 6.3).

Archivos: cerebro/registros/ronda_AAAAMMDD_HHMMSS.jsonl, una línea JSON por dato:
    {"t": 12.3, "type": "world", ...}     el mundo según la visión (5 por segundo)
    {"t": 12.4, "type": "rover", ...}     lo que reporta un rover
    {"t": 15.0, "type": "event", ...}     un cubo entró o salió de su zona, cambio de fase
    {"t": 20.1, "type": "alert", ...}     aviso del vigilante
    {"t": 61.0, "type": "summary", ...}   resumen al cerrar
"""

import json
import os
import time

import planner

WORLD_PERIOD_S = 0.2        # Cada cuánto se graba el mundo
OFFLINE_S = 1.5             # Sin reportes de un rover
STALE_S = 2.0               # Rover sin telemetría fresca
STALL_S = 15.0              # Rover que debería moverse y no avanza
STALL_CELLS = 1.5
# Estados en los que es normal estar quieto
RESTING = {"IDLE", "READY", "RUNNING", "FINISHED", "ESPERA", "FIN", "ESPERAR", "CEDER", "VERIFICAR",
           "SIN_TELEMETRIA", "PRUEBA"}


class Recorder:
    def __init__(self, directory, on_alert=None, on_status=None):
        self.directory = directory
        self.on_alert = on_alert or (lambda rover, what: None)
        self.on_status = on_status or (lambda recording, name: None)
        self.file = None
        self.name = None
        self.t0 = 0.0
        self.forced = False             # Grabación pedida a mano (pruebas fuera de ronda)
        self.phase = "IDLE"
        self.last_world_s = 0.0
        self.in_depot = {}              # color -> t en que entró a su zona
        self.rovers = {}                # id -> {seen, moved_at, pos, alerts}
        self.counts = {}

    # --- Archivo ---------------------------------------------------------------
    def _open(self, why):
        os.makedirs(self.directory, exist_ok=True)
        self.name = time.strftime("ronda_%Y%m%d_%H%M%S.jsonl")
        self.file = open(os.path.join(self.directory, self.name), "w")
        self.t0 = time.time()
        self.in_depot = {}
        self.counts = {"world": 0, "rover": 0, "event": 0, "alert": 0}
        for r in self.rovers.values():
            r["alerts"] = {}
            r["moved_at"] = time.time()
        self._write("event", what="inicio", why=why)
        self.on_status(True, self.name)

    def _close(self, why):
        if not self.file:
            return
        self._write("summary", why=why, duration=round(time.time() - self.t0, 1),
                    delivered={c: t for c, t in self.in_depot.items()}, counts=self.counts)
        self.file.close()
        self.file = None
        self.on_status(False, self.name)

    def _write(self, kind, **data):
        if not self.file:
            return
        self.counts[kind] = self.counts.get(kind, 0) + 1
        self.file.write(json.dumps({"t": round(time.time() - self.t0, 2), "type": kind, **data}) + "\n")
        self.file.flush()

    def set_forced(self, on):
        """Grabar aunque la visión esté en IDLE (para pruebas)."""
        self.forced = on
        if on and not self.file:
            self._open("manual")
        elif not on and self.file and self.phase not in ("READY", "RUNNING"):
            self._close("manual")

    # --- Entradas ---------------------------------------------------------------
    def world(self, msg):
        phase = msg.get("phase", "IDLE")
        if phase != self.phase:
            # Una ronda se graba desde READY hasta que termina
            if phase in ("READY", "RUNNING") and not self.file:
                self._open("ronda")
            self._write("event", what="fase", phase=phase)
            if phase in ("FINISHED", "IDLE") and self.file and not self.forced:
                self.phase = phase
                self._close("fin de ronda")
            self.phase = phase
        if not self.file:
            return

        now = time.time()
        if now - self.last_world_s >= WORLD_PERIOD_S:
            self.last_world_s = now
            self._write("world", phase=phase, clock=msg.get("clock"), seq=msg.get("seq"),
                        rovers=msg.get("rovers", []), cubes=msg.get("cubes", []))

        # Cubos que entran o salen de su zona
        depots = {d["color"]: d for d in msg.get("depots", [])}
        for cube in msg.get("cubes", []):
            color = cube["color"]
            if color not in depots:
                continue
            inside = planner.cube_in_depot(cube, depots[color], msg["depot_size"], msg["grid"], msg["cube_side"])
            if inside and color not in self.in_depot:
                self.in_depot[color] = round(now - self.t0, 1)
                self._write("event", what="cubo_en_zona", color=color)
            elif not inside and color in self.in_depot and cube.get("age_ms", 0) < 300:
                del self.in_depot[color]
                self._write("event", what="cubo_fuera_de_zona", color=color)

    def rover(self, status):
        now = time.time()
        rid = status.get("id")
        r = self.rovers.setdefault(rid, {"alerts": {}, "moved_at": now, "pos": None})
        pos = (status.get("col", 0.0), status.get("row", 0.0))
        if r["pos"] is None or abs(pos[0] - r["pos"][0]) + abs(pos[1] - r["pos"][1]) > STALL_CELLS:
            r["pos"], r["moved_at"] = pos, now
        if status.get("state") in RESTING:
            r["moved_at"] = now
        r["seen"] = now
        r["fresh"] = status.get("fresh", False)
        r["stale_since"] = None if r["fresh"] else (r.get("stale_since") or now)
        r["state"] = status.get("state")
        self._write("rover", **{k: v for k, v in status.items() if k not in ("t", "ip")})

    # --- Vigilante: llamar una vez por segundo ----------------------------------
    def watchdog(self):
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
                    self._write("alert", rover=rid, what=what)
                    self.on_alert(rid, what)
                elif not bad and r["alerts"].get(key):
                    r["alerts"][key] = False
                    self._write("alert", rover=rid, what=key + ": resuelto")
                    self.on_alert(rid, key + ": resuelto")
