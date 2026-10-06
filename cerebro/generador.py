"""Generador de disposiciones de práctica, con las reglas del generador oficial.

Es el algoritmo de docs/index.html (el mismo que ya está portado en sim/mundo.cpp,
official::layout), en Python y DETERMINISTA: el mismo nivel y el mismo número de
disposición dan siempre los mismos cubos. Sirve para practicar a un nivel conocido y
para repetir una ronda exacta.

    generar(0.40, 7)  ->  {"nivel": 0.4, "numero": 7,
                           "cubos": {"red": (col, row), "green": ..., "blue": ...},
                           "rovers": {10: (col, row), 11: (col, row)},
                           "metricas": {"distancia": ..., "cruces": ...}}

Las posiciones salen en CELDAS DE CANCHA (las de la visión), listas para dibujar.
No es el generador de los jueces: usa sus reglas, pero el azar es otro.
"""

import math
import random

# Todo lo que sigue está en casillas del tablero del generador (50 x 50, origen arriba a
# la izquierda del dibujo), igual que en la página oficial.
INNER_MIN, INNER_MAX, CUBE, MIN_GAP = 5, 44, 3, 2
COLORES = ["red", "green", "blue"]
GOALS = {"red": (21, 1, 8, 6), "green": (1, 21, 6, 8), "blue": (43, 21, 6, 8)}      # x, y, ancho, alto
ROBOTS = [(19, 46, 4, 3), (27, 46, 4, 3)]

# Dónde arranca cada rover en la cancha: junto a la salida (3.75, 21.5), lado a lado. El
# generador los dibuja en las filas 17.5 y 25.5; la columna es la del punto de salida
# corrida un poco hacia adentro, para que la cola del robot quede dentro de la tolerancia.
SALIDA = {10: (4.5, 17.5), 11: (4.5, 25.5)}


def a_cancha(x, y):
    """Centro en casillas del tablero -> celdas de cancha (giro de 90° y margen de 3,5)."""
    return (46.5 - y, x - 3.5)


def a_tablero(col, row):
    """Celdas de cancha -> centro en casillas del tablero."""
    return (row + 3.5, 46.5 - col)


def nominal(nivel):
    """Esquina superior izquierda alrededor de la que el generador pone cada cubo."""
    onda = math.sin(math.pi * nivel)
    return {
        "red": (23.5, 8.5 + 31 * nivel),
        "green": (8.5 + 29 * nivel, 23 + 5 * onda),
        "blue": (38.5 - 29 * nivel, 23 - 5 * onda),
    }


def _toca(a, b):
    return a[0] < b[0] + b[2] and a[0] + a[2] > b[0] and a[1] < b[1] + b[3] and a[1] + a[3] > b[1]


def _muy_cerca(a, b):
    return (a[0] < b[0] + b[2] + MIN_GAP and a[0] + a[2] + MIN_GAP > b[0] and
            a[1] < b[1] + b[3] + MIN_GAP and a[1] + a[3] + MIN_GAP > b[1])


def _rect(c):
    return (c[0], c[1], CUBE, CUBE)


def _valido(esquinas, color):
    r = _rect(esquinas[color])
    if r[0] < INNER_MIN or r[1] < INNER_MIN or r[0] + CUBE - 1 > INNER_MAX or r[1] + CUBE - 1 > INNER_MAX:
        return False
    if any(_toca(r, g) for g in GOALS.values()) or any(_toca(r, b) for b in ROBOTS):
        return False
    return not any(o != color and _muy_cerca(r, _rect(esquinas[o])) for o in esquinas)


def _punto_segmento(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    len2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2)) if len2 > 0 else 0.0
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def _segmentos(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if ((o1 > 0 > o2) or (o1 < 0 < o2)) and ((o3 > 0 > o4) or (o3 < 0 < o4)):
        return 0.0
    return min(_punto_segmento(a, c, d), _punto_segmento(b, c, d), _punto_segmento(c, a, b), _punto_segmento(d, a, b))


def metricas(esquinas):
    """Las dos medidas con que el generador juzga una disposición: distancia media de
    cada cubo a su zona (en casillas, sumando horizontal y vertical) y cuántos pares de
    trayectos cubo -> zona se cruzan o pasan a menos de 4 casillas."""
    centro = {c: (e[0] + 1.5, e[1] + 1.5) for c, e in esquinas.items()}
    meta = {c: (GOALS[c][0] + GOALS[c][2] / 2, GOALS[c][1] + GOALS[c][3] / 2) for c in esquinas}
    distancia = sum(abs(centro[c][0] - meta[c][0]) + abs(centro[c][1] - meta[c][1]) for c in esquinas) / 3
    colores = list(esquinas)
    cruces = sum(1 for i in range(len(colores)) for j in range(i + 1, len(colores))
                 if _segmentos(centro[colores[i]], meta[colores[i]], centro[colores[j]], meta[colores[j]]) < 4)
    return {"distancia": round(distancia, 1), "cruces": cruces}


def _esquinas(nivel, rng):
    lo, hi = INNER_MIN, INNER_MAX - CUBE + 1
    esperada = 6.5 + 35 * nivel
    cruces_deseados = 0 if nivel < .28 else 1 if nivel < .53 else 2 if nivel < .77 else 3
    mejor, mejor_perdida = None, 1e9
    for intento in range(700):
        variacion = 2 if nivel < .1 else (2.4 if nivel > .9 else 3.6)
        centro = nominal(nivel)
        c = {}
        for color in COLORES:
            x = centro[color][0] + rng.uniform(-variacion, variacion)
            y = centro[color][1] + rng.uniform(-variacion, variacion)
            c[color] = (min(max(round(x), lo), hi), min(max(round(y), lo), hi))
        if not all(_valido(c, color) for color in COLORES):
            continue
        m = metricas(c)
        perdida = (abs(m["distancia"] - esperada) / 36 * .55 + abs(m["cruces"] - cruces_deseados) / 3 * .45 +
                   rng.uniform(0, .06))
        if perdida < mejor_perdida:
            mejor, mejor_perdida = c, perdida
        if mejor_perdida < .035 and intento > 40:
            break
    if mejor is not None:
        return mejor
    # Respaldo del generador: posiciones al azar que sean válidas
    c = {color: (-100, -100) for color in COLORES}
    for color in COLORES:
        for _ in range(10000):
            c[color] = (rng.randint(lo, hi), rng.randint(lo, hi))
            if _valido(c, color):
                break
    return c


def generar(nivel, numero=1):
    """Disposición para ese nivel. El mismo (nivel, número) da siempre la misma."""
    nivel = round(min(max(float(nivel), 0.0), 1.0), 2)
    numero = int(numero)
    rng = random.Random(f"vision-rover/{nivel:.2f}/{numero}")
    esquinas = _esquinas(nivel, rng)
    cubos = {color: tuple(round(v, 1) for v in a_cancha(e[0] + 1.5, e[1] + 1.5)) for color, e in esquinas.items()}
    return {"nivel": nivel, "numero": numero, "cubos": cubos, "rovers": dict(SALIDA), "metricas": metricas(esquinas)}


if __name__ == "__main__":
    for nivel in (0.2, 0.3, 0.4, 0.5, 0.6):
        d = generar(nivel, 1)
        print(f"nivel {nivel:.2f}: " + "  ".join(f"{c} ({p[0]:4.1f}, {p[1]:4.1f})" for c, p in d["cubos"].items()), d["metricas"])
