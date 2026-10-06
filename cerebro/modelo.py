"""Modelo de tiempos y fallos de cada rover, ajustado con lo que pasó en la cancha.

Con la tabla de intentos (episodios.py) estima, para cada rover:
  - cuánto tarda un intento que sale bien, según lo que recorre y lo que gira;
  - qué tan probable es que el intento falle, sobre todo por girar con el cubo;
  - cuánto se pierde en un intento fallido.
De ahí sale el COSTO ESPERADO de mandar a ese rover por ese cubo de esa forma, que es
lo que usa el planificador para comparar planes (recompensa: entregar pronto;
penalización: lo que se espera perder en fallos).

    vision-system/.venv/bin/python cerebro/modelo.py           -> cerebro/datos/modelo.json

Hay pocos datos (decenas de intentos por rover), así que cada número ajustado se mezcla
con un valor inicial prudente: cuantos más intentos haya, más pesa lo medido.
"""

import json
import math
import os

import numpy as np

import episodios

HERE = os.path.dirname(os.path.abspath(__file__))
ARCHIVO = os.path.join(HERE, "datos", "modelo.json")

# Valores iniciales (segundos): lo que se usaría sin ningún dato
PREVIO = {
    "fijo": 8.0,            # Apuntar, tomar, soltar y verificar, sin contar recorridos
    "por_celda_ir": 0.13,   # ~7,5 celdas/s yendo sin cubo
    "por_celda_llevar": 0.22,   # ~4,5 celdas/s empujando
    "por_grado_sin_cubo": 0.02,
    "por_grado_con_cubo": 0.08,
}
VARIABLES = ["fijo", "por_celda_ir", "por_celda_llevar", "por_grado_sin_cubo", "por_grado_con_cubo"]
PESO_PREVIO = 6.0           # El valor inicial vale como esta cantidad de intentos medidos

# Fallos: probabilidad inicial y cuánto la sube cada cosa (se corrige con los datos)
P_FALLO_BASE = 0.15
GIRO_DIFICIL = 60.0         # Girar más que esto con el cubo es lo que más hace fallar
BORDE_CERCA = 8.0
CUBO_CERCA = 9.0

# Las rondas anteriores tenían errores de programa ya corregidos: cuentan menos
RONDA_CONFIABLE = "20261005_093200"      # Desde el firmware 1005b, con niveles del generador
PESO_RONDA_VIEJA = 0.25


def _num(v, default=0.0):
    return float(v) if v not in ("", None) else default


def _peso(ep):
    return 1.0 if ep["ronda"] >= RONDA_CONFIABLE else PESO_RONDA_VIEJA


def _fila(ep):
    return [1.0, _num(ep["d_ir"]), _num(ep["d_llevar"]), _num(ep["giro_sin_cubo"]), _num(ep["giro_con_cubo"])]


def _tiempo_util(ep):
    """Segundos del intento sin las esperas que no dependen de él (ceder el paso, etc.)."""
    return _num(ep["total"]) - _num(ep["t_espera"]) - _num(ep["t_sin_pose"])


def _ajustar_tiempo(eps):
    """Mínimos cuadrados ponderados, tirando hacia los valores iniciales."""
    previo = np.array([PREVIO[v] for v in VARIABLES])
    buenos = [e for e in eps if e["exito"] and e["giro_con_cubo"] != ""]
    if not buenos:
        return dict(PREVIO), 0
    X = np.array([_fila(e) for e in buenos])
    y = np.array([_tiempo_util(e) for e in buenos])
    w = np.array([_peso(e) for e in buenos])
    # Cada coeficiente se ancla a su valor inicial con una fuerza proporcional a la
    # escala de su variable (si no, con pocos datos salen coeficientes absurdos)
    escala = np.maximum(np.sqrt((X ** 2 * w[:, None]).sum(axis=0) / max(w.sum(), 1e-6)), 1e-3)
    ancla = np.diag(PESO_PREVIO * escala ** 2)
    A = X.T @ (X * w[:, None]) + ancla
    b = X.T @ (y * w) + ancla @ previo
    coef = np.linalg.solve(A, b)
    # Ningún término puede restar tiempo
    coef = np.maximum(coef, 0.25 * previo)
    return {v: round(float(c), 4) for v, c in zip(VARIABLES, coef)}, len(buenos)


def _tasa(eps, cond):
    """Fracción ponderada de fallos entre los intentos que cumplen 'cond', mezclada con la inicial."""
    sel = [e for e in eps if e["giro_con_cubo"] != "" and cond(e)]
    fallos = sum(_peso(e) for e in sel if not e["exito"])
    total = sum(_peso(e) for e in sel)
    return (fallos + PESO_PREVIO * P_FALLO_BASE) / (total + PESO_PREVIO), len(sel)


def _ajustar_fallos(eps):
    base, n_base = _tasa(eps, lambda e: _num(e["giro_con_cubo"]) <= GIRO_DIFICIL)
    dificil, n_dif = _tasa(eps, lambda e: _num(e["giro_con_cubo"]) > GIRO_DIFICIL)
    borde, n_borde = _tasa(eps, lambda e: _num(e["borde"], 99) < BORDE_CERCA)
    junto, n_junto = _tasa(eps, lambda e: _num(e["otro_cubo"], 99) < CUBO_CERCA)
    fallidos = [e for e in eps if not e["exito"] and e["giro_con_cubo"] != ""]
    pesos = sum(_peso(e) for e in fallidos)
    perdida = (sum(_tiempo_util(e) * _peso(e) for e in fallidos) + PESO_PREVIO * 18.0) / (pesos + PESO_PREVIO)
    return {
        "p_giro_facil": round(base, 3), "p_giro_dificil": round(max(dificil, base), 3),
        "p_borde": round(borde, 3), "p_cubo_cerca": round(junto, 3),
        "perdida_por_fallo": round(perdida, 1),
        "muestras": {"giro_facil": n_base, "giro_dificil": n_dif, "borde": n_borde, "cubo_cerca": n_junto},
    }


def ajustar(eps=None):
    eps = episodios.todos() if eps is None else eps
    modelo = {"rondas": len(set(e["ronda"] for e in eps)), "intentos": len(eps), "rovers": {}}
    for rid in (10, 11):
        mine = [e for e in eps if e["rover"] == rid]
        tiempo, n = _ajustar_tiempo(mine)
        modelo["rovers"][str(rid)] = {"tiempo": tiempo, "entregas_usadas": n, "fallos": _ajustar_fallos(mine)}
    return modelo


# --- Uso por el planificador -----------------------------------------------------------

_cache = None


def cargar():
    """El modelo guardado; si no existe todavía, los valores iniciales para los dos rovers."""
    global _cache
    if _cache is None:
        try:
            with open(ARCHIVO) as f:
                _cache = json.load(f)
        except (OSError, ValueError):
            inicial = {"tiempo": dict(PREVIO), "fallos": {
                "p_giro_facil": P_FALLO_BASE, "p_giro_dificil": 0.5, "p_borde": 0.3, "p_cubo_cerca": 0.3,
                "perdida_por_fallo": 18.0}}
            _cache = {"rovers": {"10": inicial, "11": inicial}}
    return _cache


def tiempo(rover, d_ir, d_llevar, giro_sin_cubo, giro_con_cubo, modelo=None):
    """Segundos que tarda un intento que sale bien."""
    k = (modelo or cargar())["rovers"][str(rover)]["tiempo"]
    return (k["fijo"] + k["por_celda_ir"] * d_ir + k["por_celda_llevar"] * d_llevar +
            k["por_grado_sin_cubo"] * giro_sin_cubo + k["por_grado_con_cubo"] * giro_con_cubo)


def p_fallo(rover, giro_con_cubo, borde=99.0, otro_cubo=99.0, modelo=None):
    """Probabilidad de que el intento no termine en entrega."""
    f = (modelo or cargar())["rovers"][str(rover)]["fallos"]
    # Entre 30° y GIRO_DIFICIL + 30° pasa de fácil a difícil de forma gradual
    t = min(1.0, max(0.0, (giro_con_cubo - 30.0) / GIRO_DIFICIL))
    p = f["p_giro_facil"] + t * (f["p_giro_dificil"] - f["p_giro_facil"])
    if borde < BORDE_CERCA:
        p = max(p, f["p_borde"])
    if otro_cubo < CUBO_CERCA:
        p = max(p, f["p_cubo_cerca"])
    return min(p, 0.9)


def costo(rover, d_ir, d_llevar, giro_sin_cubo, giro_con_cubo, borde=99.0, otro_cubo=99.0, modelo=None):
    """Segundos ESPERADOS hasta dejar el cubo en su zona, contando los reintentos:
    cada fallo cuesta 'perdida_por_fallo' y obliga a empezar otra vez."""
    m = modelo or cargar()
    p = p_fallo(rover, giro_con_cubo, borde, otro_cubo, m)
    perdida = m["rovers"][str(rover)]["fallos"]["perdida_por_fallo"]
    return tiempo(rover, d_ir, d_llevar, giro_sin_cubo, giro_con_cubo, m) + p / (1.0 - p) * perdida


def _validar(eps):
    """Ajusta sin las últimas rondas y predice esas: error en segundos y acierto de fallos."""
    rondas = sorted(set(e["ronda"] for e in eps))
    if len(rondas) < 6:
        return None
    prueba = set(rondas[-3:])
    m = ajustar([e for e in eps if e["ronda"] not in prueba])
    test = [e for e in eps if e["ronda"] in prueba and e["giro_con_cubo"] != ""]
    buenos = [e for e in test if e["exito"]]
    errores = [abs(tiempo(e["rover"], _num(e["d_ir"]), _num(e["d_llevar"]), _num(e["giro_sin_cubo"]),
                          _num(e["giro_con_cubo"]), m) - _tiempo_util(e)) for e in buenos]
    p_ok = [p_fallo(e["rover"], _num(e["giro_con_cubo"]), _num(e["borde"], 99), _num(e["otro_cubo"], 99), m) for e in test if e["exito"]]
    p_no = [p_fallo(e["rover"], _num(e["giro_con_cubo"]), _num(e["borde"], 99), _num(e["otro_cubo"], 99), m) for e in test if not e["exito"]]
    return {
        "rondas_de_prueba": sorted(prueba), "entregas": len(buenos),
        "error_medio_s": round(sum(errores) / len(errores), 1) if errores else None,
        "duracion_media_s": round(sum(_tiempo_util(e) for e in buenos) / len(buenos), 1) if buenos else None,
        "p_fallo_media_en_entregados": round(sum(p_ok) / len(p_ok), 2) if p_ok else None,
        "p_fallo_media_en_fallidos": round(sum(p_no) / len(p_no), 2) if p_no else None,
        "fallidos": len(p_no),
    }


if __name__ == "__main__":
    eps = episodios.todos()
    modelo = ajustar(eps)
    modelo["validacion"] = _validar(eps)
    os.makedirs(os.path.dirname(ARCHIVO), exist_ok=True)
    with open(ARCHIVO, "w") as f:
        json.dump(modelo, f, indent=2, ensure_ascii=False)
    print(f"Modelo con {modelo['intentos']} intentos de {modelo['rondas']} rondas -> {ARCHIVO}")
    for rid, m in modelo["rovers"].items():
        t, fl = m["tiempo"], m["fallos"]
        print(f"  Rover {rid} ({m['entregas_usadas']} entregas): fijo {t['fijo']:.1f} s"
              f" | ir {1 / t['por_celda_ir']:.1f} celdas/s | llevar {1 / t['por_celda_llevar']:.1f} celdas/s"
              f" | giro sin cubo {1 / t['por_grado_sin_cubo']:.0f} °/s | con cubo {1 / t['por_grado_con_cubo']:.0f} °/s")
        print(f"     fallo: giro fácil {fl['p_giro_facil']:.0%}, giro difícil {fl['p_giro_dificil']:.0%},"
              f" junto al borde {fl['p_borde']:.0%}, junto a otro cubo {fl['p_cubo_cerca']:.0%};"
              f" cada fallo cuesta {fl['perdida_por_fallo']:.0f} s   {fl['muestras']}")
    print("  Validación (ajustado sin las 3 últimas rondas):", modelo["validacion"])
