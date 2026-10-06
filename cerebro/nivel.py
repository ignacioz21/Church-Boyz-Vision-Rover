"""Clasifica una disposición de cubos: a qué nivel del generador corresponde.

El generador pone cada cubo cerca de una posición que depende solo del nivel
(generador.nominal). Dadas las tres posiciones reales, se busca el nivel que mejor las
explica. Sirve para catalogar rondas que no se generaron desde el cerebro, y como
control de que los cubos se pusieron donde indicaba la sombra.

    clasificar({"red": (col, row), "green": ..., "blue": ...})
      -> {"nivel": 0.41, "grupo": 0.4, "encaje": 2.3, "del_generador": True,
          "en_rango": True, "distancia": 21.0, "cruces": 1, "clave": "r31-20_g22-13_b22-30"}
"""

import math

import generador

GRUPOS = [0.2, 0.3, 0.4, 0.5, 0.6]      # Niveles con los que se practica
ENCAJE_MAX = 4.5                        # Error medio (casillas) por encima del cual no parece del generador
RANGO = (0.15, 0.65)


def _esquinas(cubos):
    """Celdas de cancha (centros) -> esquinas en casillas del tablero del generador."""
    out = {}
    for color, (col, row) in cubos.items():
        x, y = generador.a_tablero(col, row)
        out[color] = (x - 1.5, y - 1.5)
    return out


def _error(esquinas, nivel):
    """Distancia media (casillas) entre cada cubo y donde el generador lo pondría a ese nivel."""
    nominal = generador.nominal(nivel)
    return sum(math.hypot(e[0] - nominal[c][0], e[1] - nominal[c][1]) for c, e in esquinas.items()) / len(esquinas)


def clasificar(cubos):
    """'cubos' = {color: (col, row)} en celdas de cancha. Hacen falta los tres."""
    if set(cubos) != set(generador.COLORES):
        return None
    esquinas = _esquinas(cubos)
    nivel, encaje = min(((n / 100.0, _error(esquinas, n / 100.0)) for n in range(101)), key=lambda x: x[1])
    m = generador.metricas(esquinas)
    letra = {"red": "r", "green": "g", "blue": "b"}
    return {
        "nivel": nivel,
        "grupo": min(GRUPOS, key=lambda g: abs(g - nivel)),
        "encaje": round(encaje, 1),
        "del_generador": encaje <= ENCAJE_MAX,
        "en_rango": RANGO[0] <= nivel <= RANGO[1],
        "distancia": m["distancia"],
        "cruces": m["cruces"],
        # Misma clave = misma disposición (cubos a ~1 celda): sirve para agrupar repeticiones
        "clave": "_".join(f"{letra[c]}{round(cubos[c][0] / 2) * 2}-{round(cubos[c][1] / 2) * 2}" for c in generador.COLORES),
    }


def desvio(cubos, generada):
    """Cuánto se apartó cada cubo real de su sombra (celdas). 'generada' = generador.generar(...)."""
    return {c: round(math.hypot(cubos[c][0] - p[0], cubos[c][1] - p[1]), 1)
            for c, p in generada["cubos"].items() if c in cubos}
