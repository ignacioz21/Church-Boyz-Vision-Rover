"""Rearma el índice de rondas (registros/indice.csv) a partir de los archivos grabados.

Sirve para catalogar las rondas hechas antes de que el grabador llevara ficha, y para
reconstruir el índice si se borra. Para cada ronda calcula el nivel del generador al
que corresponde su disposición y el resultado. No modifica los archivos de ronda.

    vision-system/.venv/bin/python cerebro/catalogar.py
"""

import csv
import glob
import json
import os

import nivel
import planner
from recorder import INDEX_FIELDS

HERE = os.path.dirname(os.path.abspath(__file__))
REGISTROS = os.path.join(HERE, "registros")
# Las grabaciones viejas no guardan las zonas (son fijas en el contrato v2)
MUNDO = {"grid": {"cols": 43, "rows": 43}, "cube_side": 3.0, "depot_size": {"length": 10.0, "depth": 7.5}}
DEPOTS = {"red": {"col": 39.25, "row": 21.5}, "green": {"col": 21.5, "row": 3.75}, "blue": {"col": 21.5, "row": 39.25}}


def leer(path):
    rows = []
    with open(path) as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return rows


def fila(path):
    rows = leer(path)
    name = os.path.basename(path)
    card = next((r for r in rows if r["type"] == "ficha"), None)
    summary = next((r for r in rows if r["type"] == "summary"), {})
    worlds = [r for r in rows if r["type"] == "world"]
    statuses = [r for r in rows if r["type"] == "rover"]

    if card is None:
        # Disposición: el primer cuadro con los tres cubos antes de que algún rover trabaje
        start = next((r["t"] for r in statuses if r["state"] not in ("IDLE", "SIN_TELEMETRIA", "PRUEBA", "FIN")), None)
        first = next((w for w in worlds if len(w["cubes"]) == 3 and (start is None or w["t"] <= start + 1.0)), None)
        first = first or next((w for w in worlds if len(w["cubes"]) == 3), None)
        cubes = {c["color"]: (c["col"], c["row"]) for c in first["cubes"]} if first else {}
        info = nivel.clasificar(cubes) or {}
        ids = sorted(set(str(r["id"]) for r in statuses))
        card = {"disparador": "manual", "inicio": f"{name[6:10]}-{name[10:12]}-{name[12:14]} {name[15:17]}:{name[17:19]}:{name[19:21]}",
                "nivel": None, "numero": None, "mal_colocada": None,
                "nivel_estimado": info.get("nivel"), "grupo": info.get("grupo"), "encaje": info.get("encaje"),
                "del_generador": info.get("del_generador"), "distancia": info.get("distancia"),
                "cruces": info.get("cruces"), "clave": info.get("clave"),
                "rovers": {i: [] for i in ids},
                "firmware": {i: next((r.get("fw", "?") for r in statuses if str(r["id"]) == i), "?") for i in ids},
                "plan": {i: next((r.get("plan", 0) for r in statuses if str(r["id"]) == i), 0) for i in ids}}

    # Resultado: qué cubos terminaron dentro de su zona y cuándo entraron por última vez
    inside = {}
    for r in rows:
        if r["type"] == "event" and r.get("what") == "cubo_en_zona":
            inside[r["color"]] = r["t"]
        elif r["type"] == "event" and r.get("what") == "cubo_fuera_de_zona":
            inside.pop(r["color"], None)
    if worlds:
        final = {c["color"]: c for c in worlds[-1]["cubes"]}
        inside = {c: t for c, t in inside.items()
                  if c in final and planner.cube_in_depot(final[c], DEPOTS[c], MUNDO["depot_size"], MUNDO["grid"], MUNDO["cube_side"])}
    last = {}
    resets = 0
    for r in statuses:
        prev = last.get(r["id"])
        if prev and r.get("up", 1e9) < prev.get("up", 0):
            resets += 1
        last[r["id"]] = r
    return {
        "ronda": name[6:-6], "inicio": card.get("inicio"), "disparador": card.get("disparador"),
        "nivel": card.get("nivel"), "numero": card.get("numero"), "nivel_estimado": card.get("nivel_estimado"),
        "grupo": card.get("grupo"), "encaje": card.get("encaje"), "del_generador": card.get("del_generador"),
        "mal_colocada": card.get("mal_colocada"), "distancia": card.get("distancia"), "cruces": card.get("cruces"),
        "clave": card.get("clave"), "rovers": " ".join(sorted(card.get("rovers", {}))),
        "firmware": " ".join(f"{k}:{v}" for k, v in sorted(card.get("firmware", {}).items())),
        "plan": " ".join(f"{k}:{v}" for k, v in sorted(card.get("plan", {}).items())),
        "duracion": summary.get("duration", round(rows[-1]["t"], 1) if rows else 0),
        "entregados": len(inside), "t_tercer_cubo": round(max(inside.values()), 1) if len(inside) >= 3 else None,
        "t_red": inside.get("red"), "t_green": inside.get("green"), "t_blue": inside.get("blue"),
        "incidentes": sum(r.get("st", [0] * 5)[4] for r in last.values() if "st" in r),
        "obstaculos": sum(r.get("ob", 0) for r in last.values()), "reinicios": resets,
        "cierre": summary.get("why", "sin cierre"),
    }


def fichas():
    """{nombre de ronda: fila del índice} para todas las rondas grabadas."""
    return {os.path.basename(p)[6:-6]: fila(p) for p in sorted(glob.glob(os.path.join(REGISTROS, "ronda_*.jsonl")))}


if __name__ == "__main__":
    rows = list(fichas().values())
    with open(os.path.join(REGISTROS, "indice.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=INDEX_FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} rondas en el índice")
    print("ronda            nivel  est.  grupo encaje gen. dist cruces  entreg. 3er cubo  incid. reinic.")
    for r in rows:
        print(f"{r['ronda']}  {str(r['nivel'] or '—'):>5}  {str(r['nivel_estimado']):>4}  {str(r['grupo']):>4}  {str(r['encaje']):>5}  "
              f"{'sí' if r['del_generador'] else 'no':>3}  {str(r['distancia']):>4}  {str(r['cruces']):>3}     {r['entregados']}/3   "
              f"{str(r['t_tercer_cubo'] or '—'):>6}    {r['incidentes']:>3}   {r['reinicios']:>3}")
