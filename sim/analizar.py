#!/usr/bin/env python3
"""Corre lotes de escenarios en el simulador y analiza los resultados.

    ./analizar.py [corridas_por_lote]        (100 por defecto)

Cada escenario se corre en varios modos, con los mismos cubos:
    1 rover          el rover 10 lleva los tres cubos
    2 sin radio      los dos rovers con el plan de la PC, sin hablarse entre ellos
    2 con radio      los dos con el plan, contándose qué hace cada uno (10 % de mensajes perdidos)
    2 radio mala     igual, perdiendo el 50 % de los mensajes

Guarda cada corrida en resultados.jsonl y escribe el informe por pantalla.
"""
import json, os, statistics as st, subprocess, sys
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "cerebro"))
import planner

BIN = "/tmp/rover_sim"
COLORS = "rgb"
DEPOTS = {"r": (39.25, 21.5), "g": (21.5, 3.75), "b": (21.5, 39.25)}
SETS = [
    ("oficial D=0", {"SIM_LEVEL": "0"}),
    ("oficial D=0.25", {"SIM_LEVEL": "0.25"}),
    ("oficial D=0.5", {"SIM_LEVEL": "0.5"}),
    ("oficial D=0.75", {"SIM_LEVEL": "0.75"}),
    ("oficial D=1", {"SIM_LEVEL": "1"}),
    ("azar central", {}),
    ("azar borde", {"SIM_MARGIN": "3"}),
]
# modo -> (cantidad de rovers, variables de entorno del simulador)
MODE_ENV = {
    "1 rover": (1, {}),
    "2 sin radio": (2, {"SIM_PEER": "0"}),
    "2 con radio": (2, {"SIM_PEER": "1", "SIM_PEER_LOSS": "0.1"}),
    "2 radio mala": (2, {"SIM_PEER": "1", "SIM_PEER_LOSS": "0.5"}),
}
MODES = list(MODE_ENV)
STATES = ["IR_A_PREPARAR", "CEDER", "ESPERAR", "DESPEJAR", "HACER_LUGAR", "APUNTAR", "CAPTURAR", "REAPUNTAR", "TRANSPORTAR", "ENTREGAR", "APARTAR", "SOLTAR", "VERIFICAR"]
INCIDENTS = [("helps", "ayuda"), ("clears", "se corre"), ("parks", "aparta"), ("verify_fail", "no quedó"), ("no_progress", "sin avance"), ("nav_fail", "sin ruta"), ("aim_timeout", "no apunta"),
             ("capture_timeout", "no captura"), ("no_drop_route", "sin destino"), ("carry_fail", "ruta c/cubo"),
             ("cube_lost", "cubo perdido"), ("drop_blocked", "final tapado")]


def sim(seed, plan, env):
    return subprocess.run([BIN, str(seed), plan], env={**os.environ, **env, "SIM_JSON": "1"},
                          capture_output=True, text=True).stdout


def run(job):
    name, env, seed, mode = job
    n_rovers, mode_env = MODE_ENV[mode]
    env = {**env, **mode_env}
    if n_rovers == 1:
        plan = "P,1,10=rgb,11="
    else:
        # Lo que haría el cerebro en IDLE: mirar la cancha y repartir
        world = json.loads(subprocess.run([BIN, str(seed), "x", "--disposicion"], env={**os.environ, **env},
                                          capture_output=True, text=True).stdout)
        plan = planner.message(1, planner.plan(world))
    line = next(l for l in sim(seed, plan, env).splitlines() if l.startswith("JSON "))
    rec = json.loads(line[5:])
    rec.update(set=name, mode=mode)
    rec["all"] = rec["n_delivered"] == 3
    rec["t_official"] = max(rec["t_in"]) if rec["all"] else None    # Cuando el ÚLTIMO cubo entra en su zona
    rec["stuck"] = rec["t_end"] >= 179.9
    return rec


def pct(a, b):
    return f"{100 * a / b:.0f}%" if b else "-"


def q(values, p):
    values = sorted(values)
    return values[min(len(values) - 1, int(p * len(values)))] if values else float("nan")


def cause(r):
    """Una etiqueta corta de por qué la corrida no fue limpia."""
    if r["crash_ms"]:
        return "choque entre rovers: " + " ".join(r["crash"].split(" t=")[0].split()[:6])
    if not r["all"]:
        if r["stuck"]:
            return "atascado: " + " / ".join(f"R{v['id']} {v['final']}" for v in r["rovers"] if v["final"] != "FIN")
        whys = set()
        for i in range(3):
            if r["excess"][i] > 0:
                owners = [v for v in r["rovers"] if COLORS[i] in v["tasks"]]
                whys.add(owners[0]["why"][i] if owners else "nadie lo tenía asignado")
        return "deja cubo: " + " / ".join(sorted(whys))
    for v in r["rovers"]:
        if v["bumps"]:
            return "roza " + v["bump"].split(" t=")[0]
    for v in r["rovers"]:
        if v["out_ms"]:
            return "sale de la tolerancia en " + v["out"].split(" t=")[0]
    return "?"


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    subprocess.run([os.path.join(HERE, "run.sh"), "1"], capture_output=True)
    jobs = [(name, env, seed, mode) for name, env in SETS for seed in range(1, n + 1) for mode in MODES]
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        recs = list(pool.map(run, jobs))
    with open(os.path.join(HERE, "resultados.jsonl"), "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")

    by = defaultdict(list)
    for r in recs:
        by[(r["set"], r["mode"])].append(r)

    print(f"\n=== 1. CORRIDAS LIMPIAS ({n} escenarios por lote; mismos cubos en los tres modos) ===")
    print(f"{'lote':16}" + "".join(f"{m:>15}" for m in MODES))
    for name, _ in SETS:
        print(f"{name:16}" + "".join(f"{pct(sum(r['ok'] for r in by[(name, m)]), n):>15}" for m in MODES))
    print(f"{'TOTAL':16}" + "".join(f"{pct(sum(r['ok'] for r in recs if r['mode'] == m), n * len(SETS)):>15}" for m in MODES))

    print("\n=== 2. QUÉ PASA EN CADA MODO (todos los lotes juntos) ===")
    print(f"{'modo':13} {'limpias':>8} {'entrega 3':>10} {'2 de 3':>7} {'<=1':>5} {'choques':>8} {'roces':>6} {'fuera':>6} {'atasco':>7}")
    for m in MODES:
        rs = [r for r in recs if r["mode"] == m]
        d = Counter(r["n_delivered"] for r in rs)
        print(f"{m:13} {pct(sum(r['ok'] for r in rs), len(rs)):>8} {pct(d[3], len(rs)):>10} {pct(d[2], len(rs)):>7} "
              f"{pct(d[0] + d[1], len(rs)):>5} {pct(sum(r['crash_ms'] > 0 for r in rs), len(rs)):>8} "
              f"{pct(sum(r['bumps'] > 0 for r in rs), len(rs)):>6} {pct(sum(r['out_ms'] > 0 for r in rs), len(rs)):>6} "
              f"{pct(sum(r['stuck'] for r in rs), len(rs)):>7}")

    print("\n=== 3. TIEMPO OFICIAL: segundos hasta que entra el tercer cubo (corridas que entregan los 3) ===")
    print(f"{'lote':16}" + "".join(f"{m:>15}" for m in MODES) + "   (mediana / p90)")
    for name, _ in SETS:
        cells = []
        for m in MODES:
            ts = [r["t_official"] for r in by[(name, m)] if r["all"]]
            cells.append(f"{st.median(ts):.0f} / {q(ts, .9):.0f}" if ts else "-")
        print(f"{name:16}" + "".join(f"{c:>15}" for c in cells))

    print("\n=== 4. POR QUÉ FALLAN LAS CORRIDAS NO LIMPIAS ===")
    for m in MODES:
        rs = [r for r in recs if r["mode"] == m and not r["ok"]]
        print(f"{m}: {len(rs)} de {n * len(SETS)}")
        for tag, k in Counter(cause(r) for r in rs).most_common(8):
            print(f"    {k:3}  {tag}")

    print("\n=== 5. CHOQUES ENTRE ROVERS: en qué estaban ===")
    for m in MODES[1:]:
        rs = [r for r in recs if r["mode"] == m and r["crash_ms"]]
        print(f"{m}: {len(rs)} corridas con choque; duración mediana {st.median(r['crash_ms'] for r in rs) / 1000:.1f} s" if rs else f"{m}: ninguno")
        pairs = Counter()
        for r in rs:
            w = r["crash"].split(" t=")[0].split()        # R10 en X contra R11 en Y
            pairs[" + ".join(sorted([w[2], w[6]]))] += 1
        for k, v in pairs.most_common(6):
            print(f"    {v:3}  {k}")
        by_set = Counter(r["set"] for r in rs)
        print("    por lote: " + ", ".join(f"{name} {by_set[name]}" for name, _ in SETS))

    print("\n=== 6. EN QUÉ SE VA EL TIEMPO (corridas limpias, % del tiempo de cada rover) ===")
    print(f"{'modo':13}" + "".join(f"{s[:9]:>10}" for s in STATES) + f"{'ocioso':>10}")
    for m in MODES:
        tot = Counter()
        for r in recs:
            if r["mode"] == m and r["ok"]:
                for v in r["rovers"]:
                    for k, ms in v["state_ms"].items():
                        tot["FIN" if k in ("FIN", "ESPERA") else k] += ms
        total = sum(tot.values()) or 1
        print(f"{m:13}" + "".join(f"{100 * tot[s] / total:9.0f}%" for s in STATES) + f"{100 * tot['FIN'] / total:9.0f}%")

    print("\n=== 7. INCIDENTES POR CADA 100 CORRIDAS ===")
    print(f"{'modo':13}" + "".join(f"{label:>13}" for _, label in INCIDENTS))
    for m in MODES:
        rs = [r for r in recs if r["mode"] == m]
        print(f"{m:13}" + "".join(f"{100 * sum(v['stats'][k] for r in rs for v in r['rovers']) / len(rs):13.0f}" for k, _ in INCIDENTS))

    print("\n=== 8. QUIÉN ENTREGÓ CUÁNTO (2 rovers; entregas verificadas R10 / R11) ===")
    for m in MODES[1:]:
        split = Counter()
        for r in recs:
            if r["mode"] == m:
                split[" / ".join(str(v["stats"]["deliveries"]) for v in r["rovers"])] += 1
        print(f"{m}: " + ", ".join(f"{k}: {v}" for k, v in split.most_common(6)))

    print("\n=== 9. PRECISIÓN DE LAS ENTREGAS (distancia al centro de la zona, celdas) ===")
    along, depth = [], []
    for r in recs:
        for i, c in enumerate(COLORS):
            if r["excess"][i] == 0 and r["t_in"][i] > 0:
                x, y = r["cubes1"][i]
                dx, dy = abs(x - DEPOTS[c][0]), abs(y - DEPOTS[c][1])
                depth.append(dx if c == "r" else dy)
                along.append(dy if c == "r" else dx)
    print(f"a lo largo  (tolerancia 2.88): mediana {st.median(along):.2f}  p90 {q(along, .9):.2f}  máx {max(along):.2f}")
    print(f"en el fondo (tolerancia 1.63): mediana {st.median(depth):.2f}  p90 {q(depth, .9):.2f}  máx {max(depth):.2f}"
          f"  | a menos de 0.3 del límite: {sum(1.63 - d < 0.3 for d in depth)} de {len(depth)}")


if __name__ == "__main__":
    main()
