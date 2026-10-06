"""Grabaciones de rondas -> una fila por INTENTO DE CUBO de cada rover.

Lee cerebro/registros/ronda_*.jsonl (lo que graba recorder.py) y arma la tabla con la
que se ajusta el modelo de tiempos y fallos (modelo.py). Un intento empieza cuando el
rover sale a buscar un cubo y termina cuando lo suelta (bien o mal).

    vision-system/.venv/bin/python cerebro/episodios.py        -> cerebro/datos/episodios.csv

Solo lee archivos: no habla con los rovers ni con la visión.
"""

import csv
import glob
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REGISTROS = os.path.join(HERE, "registros")
SALIDA = os.path.join(HERE, "datos", "episodios.csv")

# La grabación no guarda las zonas (son fijas en el contrato v2)
DEPOTS = {"g": (21.5, 3.75), "r": (39.25, 21.5), "b": (21.5, 39.25)}
COLOR = {"red": "r", "green": "g", "blue": "b"}
GRID = 43.0
AXLE_OFFSET = 1.8       # El eje está esto detrás del marcador (rover_*/include/config.h)

# Estado del firmware -> fase del intento
FASE = {
    "IR_A_PREPARAR": "ir", "APUNTAR": "apuntar", "CAPTURAR": "capturar", "REAPUNTAR": "capturar",
    "TRANSPORTAR": "transportar", "HACER_LUGAR": "transportar", "ENTREGAR": "entregar", "APARTAR": "entregar",
    "SOLTAR": "soltar", "VERIFICAR": "soltar",
    "CEDER": "espera", "ESPERAR": "espera", "OBSTACULO": "obstaculo", "PLANIFICANDO": "planificar",
    "SIN_TELEMETRIA": "sin_pose",
}
FASES = ["ir", "apuntar", "capturar", "transportar", "entregar", "soltar", "espera", "obstaculo", "planificar", "sin_pose"]
CAMPOS = (["ronda", "nivel", "grupo", "firmware", "rover", "cubo", "t_inicio", "exito", "incidentes", "obstaculos", "reinicio",
           "col0", "row0", "theta0", "cubo_col", "cubo_row", "d_ir", "d_llevar", "giro_sin_cubo",
           "rumbo_captura", "giro_con_cubo", "borde", "otro_cubo", "compañero", "total"] +
          ["t_" + f for f in FASES])


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def heading(a, b):
    """Rumbo de a hacia b (0 = derecha, antihorario, la fila crece hacia abajo)."""
    return math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0])) % 360.0


def axle(r):
    th = math.radians(r["theta"])
    return (r["col"] - AXLE_OFFSET * math.cos(th), r["row"] + AXLE_OFFSET * math.sin(th))


def leer(path):
    rows = []
    with open(path) as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass                                # Última línea a medio escribir
    return rows


def episodios_de(path):
    rows = leer(path)
    ronda = os.path.basename(path)[6:-6]
    mundo = None
    abiertos = {}                                   # id de rover -> intento en curso
    ultimo = {}                                     # id de rover -> último reporte
    out = []

    def cerrar(rid, t, exito):
        ep = abiertos.pop(rid, None)
        if ep is None:
            return
        ep["exito"] = int(exito)
        ep["total"] = round(t - ep["t_inicio"], 2)
        for f in FASES:
            ep["t_" + f] = round(ep["t_" + f], 2)
        # Sin haber llegado a tomar el cubo no hay giro con cubo que medir
        if ep["rumbo_captura"] == "":
            ep["giro_con_cubo"] = ""
        out.append(ep)

    for r in rows:
        if r["type"] == "world":
            mundo = r
            continue
        if r["type"] != "rover" or "st" not in r:
            continue
        rid, t = r["id"], r["t"]
        prev = ultimo.get(rid)
        ultimo[rid] = r
        ep = abiertos.get(rid)

        # Reinicio de la placa a mitad de un intento
        if prev and r.get("up", 1e9) < prev.get("up", 0):
            if ep:
                ep["reinicio"] = 1
                cerrar(rid, t, False)
            continue
        if prev is None:
            continue
        dt = min(t - prev["t"], 3.0)

        # El tiempo del tramo anterior se le suma a la fase en que estaba
        if ep:
            fase = FASE.get(prev["state"])
            if fase:
                ep["t_" + fase] += dt
            ep["incidentes"] = r["st"][4] - ep["_inc0"]
            ep["obstaculos"] = r.get("ob", 0) - ep["_ob0"]

        tarea = r.get("task", "-")
        estado = r["state"]

        # Fin del intento: entregó (sube el contador), cambió de cubo o dejó de trabajar
        if ep:
            entrego = r["st"][0] > ep["_ent0"]
            if entrego:
                cerrar(rid, t, True)
                ep = None
            elif estado in ("IDLE", "FIN") or (tarea != ep["cubo"] and tarea != "-"):
                cerrar(rid, t, False)
                ep = None
            elif prev["state"] in ("SOLTAR",) and estado in ("IR_A_PREPARAR", "PLANIFICANDO", "ESPERAR"):
                cerrar(rid, t, False)               # Lo soltó sin entregarlo y va a reintentar
                ep = None

        # Captura: el rumbo con que quedó el cubo en las pinzas
        if ep and ep["rumbo_captura"] == "" and prev["state"] in ("CAPTURAR", "REAPUNTAR") and \
                estado in ("TRANSPORTAR", "ENTREGAR", "HACER_LUGAR", "APARTAR"):
            ep["rumbo_captura"] = round(r["theta"], 1)
            ep["giro_con_cubo"] = round(abs(wrap(heading((ep["cubo_col"], ep["cubo_row"]), DEPOTS[ep["cubo"]]) - r["theta"])), 1)

        # Comienzo de un intento
        if ep is None and estado in ("IR_A_PREPARAR", "APUNTAR") and tarea in DEPOTS and mundo and r.get("fresh"):
            cubo = next((c for c in mundo["cubes"] if COLOR.get(c["color"]) == tarea), None)
            if cubo is None:
                continue
            pos, cpos, depot = axle(r), (cubo["col"], cubo["row"]), DEPOTS[tarea]
            otros = [math.hypot(c["col"] - cpos[0], c["row"] - cpos[1]) for c in mundo["cubes"] if COLOR.get(c["color"]) != tarea]
            peer = [x for x in mundo["rovers"] if x["id"] != rid]
            ep = {k: "" for k in CAMPOS}
            ep.update({
                "ronda": ronda, "rover": rid, "cubo": tarea, "t_inicio": round(t, 2), "reinicio": 0,
                "incidentes": 0, "obstaculos": 0,
                "col0": round(pos[0], 1), "row0": round(pos[1], 1), "theta0": round(r["theta"], 1),
                "cubo_col": round(cpos[0], 1), "cubo_row": round(cpos[1], 1),
                "d_ir": round(math.hypot(cpos[0] - pos[0], cpos[1] - pos[1]), 1),
                "d_llevar": round(math.hypot(depot[0] - cpos[0], depot[1] - cpos[1]), 1),
                "giro_sin_cubo": round(abs(wrap(heading(pos, cpos) - r["theta"])), 1),
                "borde": round(min(cpos[0], GRID - cpos[0], cpos[1], GRID - cpos[1]), 1),
                "otro_cubo": round(min(otros), 1) if otros else 99.0,
                "compañero": round(math.hypot(peer[0]["col"] - pos[0], peer[0]["row"] - pos[1]), 1) if peer else 99.0,
                "_inc0": r["st"][4], "_ob0": r.get("ob", 0), "_ent0": r["st"][0],
            })
            for f in FASES:
                ep["t_" + f] = 0.0
            abiertos[rid] = ep

    for rid in list(abiertos):
        cerrar(rid, ultimo[rid]["t"], False)
    return out


def todos():
    """Todos los intentos, cada uno con el nivel de su ronda y el firmware de su rover."""
    import catalogar                    # Acá, para no importar el grabador si no hace falta
    fichas = catalogar.fichas()
    eps = []
    for path in sorted(glob.glob(os.path.join(REGISTROS, "ronda_*.jsonl"))):
        ronda = os.path.basename(path)[6:-6]
        ficha = fichas.get(ronda, {})
        firmware = dict(x.split(":", 1) for x in (ficha.get("firmware") or "").split() if ":" in x)
        for ep in episodios_de(path):
            # El nivel generado si lo hay; si no, el estimado por las posiciones
            ep["nivel"] = ficha.get("nivel") if ficha.get("nivel") not in (None, "") else ficha.get("nivel_estimado")
            ep["grupo"] = ficha.get("grupo")
            ep["firmware"] = firmware.get(str(ep["rover"]), "?")
            eps.append(ep)
    return eps


if __name__ == "__main__":
    eps = todos()
    os.makedirs(os.path.dirname(SALIDA), exist_ok=True)
    with open(SALIDA, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CAMPOS, extrasaction="ignore")
        w.writeheader()
        w.writerows(eps)
    ok = sum(e["exito"] for e in eps)
    print(f"{len(eps)} intentos de cubo en {len(set(e['ronda'] for e in eps))} rondas -> {SALIDA}")
    print(f"  entregados: {ok}  fallidos: {len(eps) - ok}")
    for rid in sorted(set(e["rover"] for e in eps)):
        mine = [e for e in eps if e["rover"] == rid]
        good = [e for e in mine if e["exito"]]
        media = sum(e["total"] for e in good) / len(good) if good else 0.0
        print(f"  Rover {rid}: {len(mine)} intentos, {len(good)} entregados, {media:.1f} s por entrega lograda")
