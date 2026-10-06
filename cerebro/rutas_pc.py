"""Rutas calculadas en la PC con el planificador del propio rover (fase CEREBRO).

El firmware de cada rover se compila como biblioteca (compilar.sh, igual que hace el
simulador) y acá se le pregunta, para un cubo: cómo lo tomaría ESE rover desde una pose
dada, por dónde iría, cómo lo entregaría y dónde quedaría al terminar. Es el mismo
código que corre a bordo, así que una ruta que sale de acá es una ruta que el rover
habría elegido solo; la diferencia es que la calcula la PC antes de READY (reglamento
6.2.5 y 8.6.6) en vez del rover con el reloj corriendo.

    preplan(10, (col, row, rumbo) del EJE, "green", mundo)  ->  dict con captura y rutas, o None
    eje(10, col, row, rumbo) del marcador                    ->  (col, row, rumbo) del eje

No abre sockets ni habla con nadie.
"""

import ctypes
import glob
import math
import os
import subprocess
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
LIB_DIR = os.path.join(HERE, "datos")
RAIZ = os.path.join(HERE, "..")
COLORES = ["red", "green", "blue"]          # Orden de CubeColor en el firmware
MAX_TRAMOS = 12

_libs = {}
_lock = threading.Lock()                    # La biblioteca tiene estado propio: una llamada por vez


def _vieja(rid, path):
    """¿Alguna fuente del rover (o del empaquetado) es más nueva que la biblioteca?"""
    if not os.path.exists(path):
        return True
    built = os.path.getmtime(path)
    fuentes = glob.glob(os.path.join(RAIZ, f"rover_{rid}", "src", "*.cpp")) + \
        glob.glob(os.path.join(RAIZ, f"rover_{rid}", "include", "*.h")) + \
        [os.path.join(RAIZ, "sim", "rover_lib.cpp")]
    return any(os.path.getmtime(f) > built for f in fuentes)


def _lib(rid):
    if rid not in _libs:
        path = os.path.join(LIB_DIR, f"librover_{rid}.so")
        if _vieja(rid, path):
            subprocess.run([os.path.join(HERE, "compilar.sh")], check=True, capture_output=True)
        lib = ctypes.CDLL(path)
        lib.rover_preplan.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        lib.rover_preplan.restype = ctypes.c_int
        lib.rover_axle_offset.restype = ctypes.c_float
        lib.rover_id.restype = ctypes.c_int
        if lib.rover_id() != rid:
            raise RuntimeError(f"{path} es del rover {lib.rover_id()}, no del {rid}")
        _libs[rid] = lib
    return _libs[rid]


def disponible(rid):
    """¿Se puede planificar para ese rover? (False si no compila o no hay compilador.)"""
    try:
        _lib(rid)
        return True
    except (OSError, subprocess.SubprocessError, RuntimeError, AttributeError):
        return False


def eje(rid, col, row, rumbo):
    """Pose del marcador (lo que publica la visión) -> pose del eje (con la que planifica el rover)."""
    with _lock:
        offset = _lib(rid).rover_axle_offset()
    th = math.radians(rumbo)
    return (col - offset * math.cos(th), row + offset * math.sin(th), rumbo)


def preplan(rid, pose, color, mundo):
    """Cómo atendería el rover 'rid' el cubo 'color' saliendo de 'pose' = (col, row, rumbo) del eje.

    mundo = {"grid": (cols, rows), "zona": (largo, fondo), "lado": lado del cubo,
             "cubos": {color: (col, row)}, "zonas": {color: (col, row)},
             "otro": (col, row, rumbo) del marcador del compañero, o None}
    Devuelve None si no hay forma, o:
      {"color", "cubo", "desde", "captura", "dir", "ida": [(col, row, atrás)], "con_entrega",
       "entrega", "dir_entrega", "con_cubo": [(col, row)], "fin": (col, row, rumbo), "calculos"}
    """
    w = (ctypes.c_float * 30)()
    w[0], w[1] = mundo["grid"]
    w[2], w[3] = mundo["zona"]
    w[4] = mundo["lado"]
    w[5], w[6], w[7] = pose
    otro = mundo.get("otro")
    if otro:
        w[8], w[9], w[10], w[11] = 1.0, otro[0], otro[1], otro[2]
    for i, c in enumerate(COLORES):
        if c in mundo["cubos"]:
            w[12 + 3 * i], w[13 + 3 * i], w[14 + 3 * i] = 1.0, mundo["cubos"][c][0], mundo["cubos"][c][1]
        if c in mundo["zonas"]:
            w[21 + 2 * i], w[22 + 2 * i] = mundo["zonas"][c]
    out = (ctypes.c_float * 80)()
    with _lock:
        ok = _lib(rid).rover_preplan(w, COLORES.index(color), out)
    if not ok:
        return None
    n_ida, n_cubo = int(out[7]), int(out[48])
    return {
        "color": color,
        "cubo": (out[0], out[1]), "desde": (out[2], out[3]),
        "captura": (out[4], out[5]), "dir": int(out[6]),
        "ida": [(out[8 + 3 * i], out[9 + 3 * i], out[10 + 3 * i] > 0.5) for i in range(n_ida)],
        "con_entrega": out[44] > 0.5, "entrega": (out[45], out[46]), "dir_entrega": int(out[47]),
        "con_cubo": [(out[49 + 2 * i], out[50 + 2 * i]) for i in range(n_cubo)],
        "fin": (out[73], out[74], out[75]), "calculos": int(out[76]),
    }
