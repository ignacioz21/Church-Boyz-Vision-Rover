#!/usr/bin/env python3
"""Corre rondas del simulador con la fase CEREBRO: para cada escenario la PC calcula el
plan completo (cerebro/planner.py, plan_completo) y cada rover recibe el suyo, como en
la cancha antes de READY. Compara contra las mismas rondas sin plan de la PC.

    ./con_cerebro.py [semillas por nivel] [niveles...]      p. ej.  ./con_cerebro.py 40 0.2 0.4 0.6

Usa el último binario de run.sh (/tmp/rover_sim) y las bibliotecas de SIM_LIB_DIR.
"""
import json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "cerebro"))
import planner

BIN = os.environ.get("SIM_BIN", "/tmp/rover_sim")
START = os.environ.get("SIM_START", "3.1,19.2,2.9,27.5")       # Salida real: lado a lado


def run(job):
    level, seed, mode = job
    env = {**os.environ, "SIM_LEVEL": str(level), "SIM_START": START, "SIM_JSON": "1", "SIM_PLAN_COUNT": "1"}
    info = ""
    if mode == "cerebro":
        world = json.loads(subprocess.run([BIN, str(seed), "x", "--disposicion"], env=env, capture_output=True, text=True).stdout)
        plan = planner.plan_completo(world, 1)
        if plan:
            for rid, msg in plan["mensajes"].items():
                env[f"SIM_PLAN_{rid}"] = msg
            info = " ".join(f"{r}={t}" for r, t in plan["tareas"].items()) + (" juntos" if plan["juntos"] else "")
    out = subprocess.run([BIN, str(seed), "ninguno"], env=env, capture_output=True, text=True)
    res = json.loads(next(l for l in out.stdout.splitlines() if l.startswith("JSON "))[5:])
    res["travel_plans"] = sum(int(l.split()[3]) for l in out.stderr.splitlines() if l.startswith("PLAN"))
    res["all_plans"] = sum(int(l.split()[2]) for l in out.stderr.splitlines() if l.startswith("PLAN"))
    res["crash"] = "primer choque" in out.stdout
    res.update(level=level, seed=seed, mode=mode, info=info)
    return res


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    levels = [float(x) for x in sys.argv[2:]] or [0.2, 0.3, 0.4, 0.5, 0.6]
    jobs = [(lv, s, m) for lv in levels for s in range(1, n + 1) for m in ("a bordo", "cerebro")]
    with ThreadPoolExecutor(1 if os.environ.get("SERIE") else 4) as pool:      # plan_completo se serializa solo (candado)
        results = list(pool.map(run, jobs))
    for lv in levels + ["todos"]:
        for mode in ("a bordo", "cerebro"):
            rs = [r for r in results if r["mode"] == mode and (lv == "todos" or r["level"] == lv)]
            ok = [r for r in rs if r["ok"]]
            done = [r for r in rs if r["n_delivered"] == 3]
            t = sum(r["t_end"] for r in done) / max(len(done), 1)
            print(f"nivel {lv!s:5} {mode:8} limpias {len(ok):3}/{len(rs)}  entregó todos {len(done):3}  choques {sum(r['crash'] for r in rs):2}  "
                  f"tiempo {t:5.1f} s  cálculos de ruta por ronda: {sum(r['all_plans'] for r in rs) / len(rs):5.1f} (en viaje {sum(r['travel_plans'] for r in rs) / len(rs):4.1f})")
    if os.environ.get("DETALLE"):
        for r in results:
            if r["mode"] == "cerebro":
                print(r["level"], r["seed"], r["info"], "OK" if r["ok"] else "FALLO", r["n_delivered"], r["t_end"])


if __name__ == "__main__":
    main()
