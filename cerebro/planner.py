"""Planificador de la PC: reparte los cubos entre los rovers y les calcula las rutas.

Se usa ANTES de READY (reglamento 6.2.5 y 8.6.6), en la fase CEREBRO:

    plan_completo(world, plan_id)   reparto + orden + captura y rutas de cada cubo, y el
                                    mensaje que se carga en cada rover. Las rutas salen del
                                    planificador del propio rover (rutas_pc.py).
    plan(world)                     solo el reparto y el orden, estimado con modelo.py
                                    (lo usa el simulador; las rutas las calcula cada rover).

    world = mensaje de la visión (dict)

Funciones puras: no abren sockets ni leen nada.
"""

import itertools
import math

import modelo

LETTER = {"red": "r", "green": "g", "blue": "b"}

# Los tiempos y las probabilidades de fallo de cada rover salen de modelo.py, que se
# ajusta con las rondas grabadas en la cancha. Acá quedan solo las penalizaciones que
# dependen de los DOS rovers a la vez.
T_CROSSING = 4.0        # castigo cuando los dos rovers pasarían por el mismo lugar a la vez
BLOCK_RADIUS = 9.0      # un cubo a menos de esto del centro de una zona la deja ocupada
STAGE_DIST = 12.0       # desde dónde se encara un cubo (rover_*/include/config.h, STAGE_DIST_MIN + 1)
CUBE_AHEAD = 6.4        # eje -> centro del cubo en las pinzas
BODY = 3.5              # medio ancho del rover, para saber si un punto de preparación cabe
GRID_TOL = 2.0          # lo que el rover puede salirse de las líneas


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def cube_in_depot(cube, depot, depot_size, grid, cube_side):
    """¿El cubo entero está dentro de su zona? (regla oficial, CONTRATO.md sección 3)"""
    sides = {
        "top": depot["row"], "bottom": grid["rows"] - depot["row"],
        "left": depot["col"], "right": grid["cols"] - depot["col"],
    }
    side = min(sides, key=sides.get)
    if side in ("top", "bottom"):
        semi_col, semi_row = depot_size["length"] / 2, depot_size["depth"] / 2
    else:
        semi_col, semi_row = depot_size["depth"] / 2, depot_size["length"] / 2
    margin = cube_side * math.sqrt(2) / 2
    return (abs(cube["col"] - depot["col"]) <= semi_col - margin and
            abs(cube["row"] - depot["row"]) <= semi_row - margin)


def _seg_distance(a, b, c, d):
    """Distancia mínima entre los segmentos a-b y c-d."""
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def point_seg(p, q, r):
        dx, dy = r[0] - q[0], r[1] - q[1]
        len2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((p[0] - q[0]) * dx + (p[1] - q[1]) * dy) / len2)) if len2 else 0.0
        return math.hypot(p[0] - (q[0] + t * dx), p[1] - (q[1] + t * dy))

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return 0.0
    return min(point_seg(a, c, d), point_seg(b, c, d), point_seg(c, a, b), point_seg(d, a, b))


def _heading(a, b):
    """Rumbo de a hacia b en grados (0 = derecha, antihorario; la fila crece hacia abajo)."""
    return math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0])) % 360.0


def _wrap(deg):
    return abs((deg + 180.0) % 360.0 - 180.0)


def best_capture(rid, pos, heading, color, cubes, depots, grid):
    """Mejor forma de que el rover 'rid', parado en 'pos' mirando a 'heading', tome el cubo
    'color' y lo lleve a su zona. Prueba las 8 direcciones de la rejilla del firmware y
    también ir derecho desde donde está. Devuelve (costo esperado en s, dirección 0-7 o
    -1 = derecho, rumbo con que queda el cubo en las pinzas)."""
    cube, depot = cubes[color], depots[color]
    border = min(cube[0], grid["cols"] - cube[0], cube[1], grid["rows"] - cube[1])
    others = [_dist(cube, p) for c, p in cubes.items() if c != color]
    near = min(others) if others else 99.0
    to_depot = _heading(cube, depot)
    d_carry = _dist(cube, depot)

    options = []
    # Derecho desde donde está (si el cubo queda razonablemente enfrente y no muy cerca)
    direct = _heading(pos, cube)
    if _dist(pos, cube) > 10.0:
        options.append((-1, direct, _dist(pos, cube) - CUBE_AHEAD, _wrap(direct - heading)))
    for d in range(8):
        h = d * 45.0
        stage = (cube[0] - STAGE_DIST * math.cos(math.radians(h)), cube[1] + STAGE_DIST * math.sin(math.radians(h)))
        # El punto de preparación tiene que caber en la cancha y no caer sobre otro cubo
        if not (-GRID_TOL + BODY <= stage[0] <= grid["cols"] + GRID_TOL - BODY and
                -GRID_TOL + BODY <= stage[1] <= grid["rows"] + GRID_TOL - BODY):
            continue
        if any(_dist(stage, p) < 6.0 for c, p in cubes.items() if c != color):
            continue
        travel = _dist(pos, stage)
        turn = _wrap(_heading(pos, stage) - heading) + _wrap(h - _heading(pos, stage)) if travel > 2.0 else _wrap(h - heading)
        options.append((d, h, travel + STAGE_DIST - CUBE_AHEAD, turn))

    best = None
    for d, h, d_go, turn_free in options:
        cost = modelo.costo(rid, d_go, d_carry, turn_free, _wrap(to_depot - h), border, near)
        if best is None or cost < best[0]:
            best = (cost, d, h)
    if best is None:                                    # Nada cabe: se estima yendo derecho igual
        best = (modelo.costo(rid, _dist(pos, cube), d_carry, _wrap(direct - heading), _wrap(to_depot - direct), border, near), -1, direct)
    return best


def _timeline(rid, start, heading, order, cubes, depots, grid):
    """Recorrido de un rover: [(color, t_inicio, t_toma, t_entrega, tramos, dirección)]."""
    t, pos, out = 0.0, start, []
    remaining = dict(cubes)
    for color in order:
        cube, depot = cubes[color], depots[color]
        cost, direction, h = best_capture(rid, pos, heading, color, remaining, depots, grid)
        # El costo esperado se reparte: la parte de ir hasta el cubo y la de llevarlo
        share = _dist(pos, cube) / max(_dist(pos, cube) + _dist(cube, depot), 1e-6)
        t_pick = t + cost * min(max(share, 0.25), 0.75)
        t_drop = t + cost
        out.append((color, t, t_pick, t_drop, [(pos, cube), (cube, depot)], direction))
        t, pos, heading = t_drop, depot, _heading(cube, depot)
        remaining.pop(color, None)                      # Ya no estorba a los siguientes
    return out


def _cost(assignment, starts, cubes, depots, blockers, headings=None, grid=None, detail=None):
    """Tiempo estimado hasta el último cubo, con castigos por estorbos."""
    headings = headings or {}
    grid = grid or {"cols": 43.0, "rows": 43.0}
    lines = {rid: _timeline(rid, starts[rid], headings.get(rid, 0.0), order, cubes, depots, grid)
             for rid, order in assignment.items()}
    if detail is not None:
        detail.update({rid: [(c, d, round(t1, 1)) for c, _, _, t1, _, d in line] for rid, line in lines.items()})
    pick = {c: t_pick for line in lines.values() for c, _, t_pick, _, _, _ in line}
    owner = {c: rid for rid, line in lines.items() for c, _, _, _, _, _ in line}

    finish, penalty = 0.0, 0.0
    for rid, line in lines.items():
        delay = 0.0
        for color, _, t_pick, t_drop, _, _ in line:
            # Una zona ocupada por otro cubo: no se puede entregar hasta que lo saquen
            for other in blockers.get(color, ()):
                if other not in pick:
                    continue
                if owner[other] == rid and pick[other] > t_pick:
                    penalty += 30.0                     # El mismo rover lo llevaría después: orden imposible
                else:
                    delay = max(delay, pick[other] - (t_drop + delay))
            finish = max(finish, t_drop + delay)

    # Los dos rovers pasando por el mismo lugar al mismo tiempo: uno tendrá que esperar
    rids = list(lines)
    if len(rids) == 2:
        for _, a0, _, a1, segs_a, _ in lines[rids[0]]:
            for _, b0, _, b1, segs_b, _ in lines[rids[1]]:
                same_time = a0 < b1 and b0 < a1
                if same_time and any(_seg_distance(*sa, *sb) < 8.0 for sa in segs_a for sb in segs_b):
                    penalty += T_CROSSING
    total = sum(line[-1][3] for line in lines.values() if line)
    return finish + penalty + 0.05 * total


def candidates(world, top=5):
    """Los mejores planes para esta disposición, del mejor al peor. Cada uno:
    {"tareas": {10: "gb", 11: "r"}, "costo": segundos esperados (con penalizaciones),
     "detalle": {10: [(color, dirección de captura, t de entrega), ...], ...}}"""
    rovers = {r["id"]: (r["col"], r["row"]) for r in world.get("rovers", [])}
    headings = {r["id"]: r.get("theta", 0.0) for r in world.get("rovers", [])}
    depots = {d["color"]: (d["col"], d["row"]) for d in world.get("depots", [])}
    grid = world["grid"]
    pending = {}
    for c in world.get("cubes", []):
        color = c["color"]
        if color not in depots or not (0 <= c["col"] <= grid["cols"] and 0 <= c["row"] <= grid["rows"]):
            continue                                    # Sin zona, o un fantasma fuera de la cancha
        depot = {"col": depots[color][0], "row": depots[color][1]}
        if cube_in_depot(c, depot, world["depot_size"], grid, world["cube_side"]):
            continue                                    # Ya está entregado
        pending[color] = (c["col"], c["row"])
    if not rovers:
        return []
    if not pending:
        return [{"tareas": {rid: "" for rid in rovers}, "costo": 0.0, "detalle": {}}]

    # Qué cubos dejan ocupada la zona de otro color
    blockers = {color: [other for other, pos in pending.items()
                        if other != color and _dist(pos, depots[color]) < BLOCK_RADIUS]
                for color in pending}

    rids = sorted(rovers)
    colors = sorted(pending)
    found = []
    for owners in itertools.product(rids, repeat=len(colors)):
        groups = {rid: [c for c, o in zip(colors, owners) if o == rid] for rid in rids}
        for orders in itertools.product(*(itertools.permutations(g) for g in groups.values())):
            assignment = dict(zip(groups, orders))
            detail = {}
            cost = _cost(assignment, rovers, pending, depots, blockers, headings, grid, detail)
            found.append({"tareas": {rid: "".join(LETTER[c] for c in order) for rid, order in assignment.items()},
                          "costo": round(cost, 1), "detalle": detail})
    found.sort(key=lambda p: p["costo"])
    return found[:top]


def plan(world):
    """Reparto y orden de los cubos. Devuelve {id_rover: "letras en orden"}."""
    best = candidates(world, top=1)
    return best[0]["tareas"] if best else {}


def message(plan_id, tasks):
    """Mensaje para los rovers (rover_*/include/plan.h): "P,<id>,10=gb,11=r"."""
    return "P,%d,%s" % (plan_id, ",".join("%d=%s" % (rid, tasks[rid]) for rid in sorted(tasks)))


# --- Fase CEREBRO: reparto + rutas reales --------------------------------------------

LETRA_COLOR = {v: k for k, v in LETTER.items()}
# Para salir a la vez los dos primeros recorridos tienen que estar MUY separados: a bordo,
# un rover no pivota con el compañero en movimiento a menos de ~17 celdas (su radio de
# resguardo más lo que barren las pinzas). Con 7,5 el rover 10 quedó 12 s sin ruta.
CLEAR_GAP = 17.0
CROSS_GAP = 6.0         # Más cerca que esto, los recorridos de los dos rovers se estorban
T_CRUCE = 20.0          # Segundos que se le cargan a un reparto que cruza a los rovers


def _polilinea(pre):
    """Puntos del recorrido completo de un cubo: salida, ida, cubo, con cubo, zona."""
    pts = [pre["desde"]] + [(x, y) for x, y, _ in pre["ida"]] + [pre["captura"], pre["cubo"]]
    pts += list(pre["con_cubo"])
    if pre["con_entrega"]:
        pts.append(pre["entrega"])
    return pts


def _medidas(pre, rumbo, zona):
    """(celdas sin cubo, celdas con cubo, giro sin cubo, giro con cubo) del recorrido 'pre'."""
    d_ir = d_llevar = giro = giro_cubo = 0.0
    pos, h = pre["desde"], rumbo
    for x, y, atras in pre["ida"]:
        if _dist(pos, (x, y)) < 0.1:
            continue
        hd = _heading(pos, (x, y)) if not atras else _heading((x, y), pos)
        giro += _wrap(hd - h)
        d_ir += _dist(pos, (x, y))
        pos, h = (x, y), hd
    apunta = _heading(pre["captura"], pre["cubo"])
    giro += _wrap(apunta - h)
    d_ir += max(0.0, _dist(pre["captura"], pre["cubo"]) - CUBE_AHEAD)
    # Con el cubo: desde donde queda tomado
    rad = math.radians(apunta)
    pos = (pre["cubo"][0] - CUBE_AHEAD * math.cos(rad), pre["cubo"][1] + CUBE_AHEAD * math.sin(rad))
    h = apunta
    for p in list(pre["con_cubo"]) + [zona]:
        if _dist(pos, p) < 0.5:
            continue
        hd = _heading(pos, p)
        giro_cubo += _wrap(hd - h)
        d_llevar += _dist(pos, p)
        pos, h = p, hd
    return d_ir, max(0.0, d_llevar - CUBE_AHEAD), giro, giro_cubo


def _segundos(rid, pre, rumbo, zona, cubos, grid):
    """Segundos de ese recorrido: lo que hay que andar y girar, sin y con el cubo.

    Se usan los mismos números para los dos rovers (los valores iniciales de modelo.py) y
    no el ajuste por rover: con las rondas grabadas hasta hoy ese ajuste predice mal, y
    una diferencia inventada entre rovers basta para cruzarlos (mandar al de abajo por
    el cubo de arriba), que es lo que más tiempo cuesta."""
    d_ir, d_llevar, giro, giro_cubo = _medidas(pre, rumbo, zona)
    k = modelo.PREVIO
    return (k["fijo"] + k["por_celda_ir"] * d_ir + k["por_celda_llevar"] * d_llevar +
            k["por_grado_sin_cubo"] * giro + k["por_grado_con_cubo"] * giro_cubo)


def _separacion(a, b):
    """Distancia mínima entre dos polilíneas."""
    return min(_seg_distance(a[i], a[i + 1], b[j], b[j + 1])
               for i in range(len(a) - 1) for j in range(len(b) - 1))


def _seccion(pre):
    """Sección del mensaje para un cubo (rover_*/include/plan.h); números en décimas de celda."""
    d = lambda v: str(int(round(v * 10)))
    campos = [LETTER[pre["color"]], d(pre["cubo"][0]), d(pre["cubo"][1]), d(pre["desde"][0]), d(pre["desde"][1]),
              d(pre["captura"][0]), d(pre["captura"][1]), str(pre["dir"]), str(len(pre["ida"]))]
    for x, y, atras in pre["ida"]:
        campos += [d(x), d(y), "1" if atras else "0"]
    campos += ["1" if pre["con_entrega"] else "0", d(pre["entrega"][0]), d(pre["entrega"][1]),
               str(pre["dir_entrega"]), str(len(pre["con_cubo"]))]
    for x, y in pre["con_cubo"]:
        campos += [d(x), d(y)]
    return ",".join(campos)


def message_completo(plan_id, tareas, rutas, juntos):
    """Mensaje para UN rover: reparto de los dos + las rutas de ese rover + suma de control."""
    cuerpo = message(plan_id, tareas) + (",x=1" if juntos else "")
    for pre in rutas:
        cuerpo += "|" + _seccion(pre)
    if not rutas:
        return cuerpo
    return cuerpo + "|K%d" % (sum(cuerpo.encode()) & 0xFFFF)


def plan_completo(world, plan_id=1, rovers=None):
    """Reparto, orden y rutas para la disposición de 'world'. 'rovers' = ids a planificar
    (por defecto, todos los que ve la visión).

    Regla: cada rover empieza por UN cubo; el que queda es el comodín, va al final de la
    lista de los dos (con su ruta desde donde cada uno termina) y lo toma el que quede
    libre primero. Entre los repartos posibles se elige el que antes termina, y a igualdad
    el que entrega antes los primeros cubos (los fáciles y cercanos primero).

    Devuelve None si no hay nada que planificar, o:
      {"id", "tareas": {rid: "gb"}, "rutas": {rid: [preplan, ...]}, "juntos": bool,
       "costo": s, "mensajes": {rid: "P,..."}, "comodin": color o None}
    """
    import rutas_pc                         # Acá: el simulador usa plan() sin compilar nada

    grid = world["grid"]
    zonas = {d["color"]: (d["col"], d["row"]) for d in world.get("depots", [])}
    visibles = {r["id"]: (r["col"], r["row"], r.get("theta", 0.0)) for r in world.get("rovers", [])}
    poses = {rid: p for rid, p in visibles.items() if rovers is None or rid in rovers}
    cubos = {}
    for c in world.get("cubes", []):
        color = c["color"]
        if color not in zonas or not (0 <= c["col"] <= grid["cols"] and 0 <= c["row"] <= grid["rows"]):
            continue
        if cube_in_depot(c, {"col": zonas[color][0], "row": zonas[color][1]}, world["depot_size"], grid, world["cube_side"]):
            continue
        cubos[color] = (c["col"], c["row"])
    rids = sorted(r for r in poses if rutas_pc.disponible(r))
    if not rids or not cubos:
        return None

    base = {"grid": (grid["cols"], grid["rows"]), "zona": (world["depot_size"]["length"], world["depot_size"]["depth"]),
            "lado": world["cube_side"], "zonas": zonas}
    ejes = {rid: rutas_pc.eje(rid, *poses[rid]) for rid in rids}

    def primero(rid, color):
        """El rover sale de donde está, con todo en su lugar y el compañero todavía quieto."""
        otro = next((p for o, p in visibles.items() if o != rid), None)     # Estorba aunque no se planifique para él
        return rutas_pc.preplan(rid, ejes[rid], color, dict(base, cubos=cubos, otro=otro))

    def siguiente(rid, desde, color, entregados):
        """Sale de donde quedó tras entregar; los cubos ya entregados están en sus zonas."""
        mundo = {c: (zonas[c] if c in entregados else p) for c, p in cubos.items()}
        return rutas_pc.preplan(rid, desde, color, dict(base, cubos=mundo, otro=None))

    colores = sorted(cubos)
    mejor = None
    for propios in itertools.permutations(colores, min(len(rids), len(colores))):
        inicial = dict(zip(rids, propios))              # Primer cubo de cada rover
        pre1 = {rid: primero(rid, c) for rid, c in inicial.items()}
        # Un cubo que hoy no se puede llevar (su zona tapada por otro, p. ej.) queda en la
        # lista sin ruta: el rover lo resuelve a bordo cuando se destape. Cuenta caro.
        SIN_RUTA = 150.0                # Solo gana si ningún reparto tiene ruta para los dos
        t1 = {rid: _segundos(rid, pre1[rid], poses[rid][2], zonas[inicial[rid]], cubos, grid) if pre1[rid] else SIN_RUTA
              for rid in inicial}
        pre1 = {rid: p for rid, p in pre1.items() if p}
        resto = [c for c in colores if c not in propios]
        listas = {rid: [pre1[rid]] if rid in pre1 else [] for rid in rids}
        orden = {rid: [inicial[rid]] if rid in inicial else [] for rid in rids}
        fin = dict(t1)
        comodin = None
        # Los que sobran: de a uno, cada vez para el rover que quede libre primero. Solo el
        # primero lleva ruta (es el que se sabe desde dónde se sale); a los dos se les da.
        entregados = set(propios)
        for n, color in enumerate(sorted(resto, key=lambda c: min(_dist(cubos[c], zonas[c]) for _ in [0]))):
            libre = min(fin, key=fin.get) if fin else None
            for rid in rids:
                orden[rid].append(color)
                if n == 0 and listas[rid]:
                    pre = siguiente(rid, listas[rid][0]["fin"], color, entregados)
                    if pre:
                        listas[rid].append(pre)
                        if rid == libre:
                            fin[rid] += _segundos(rid, pre, listas[rid][0]["fin"][2], zonas[color], cubos, grid)
                    elif rid == libre:
                        fin[rid] += 40.0                # Sin ruta conocida: lo resolverá a bordo, caro
            if n == 0:
                comodin = color
            elif libre is not None:
                fin[libre] += 30.0
            entregados.add(color)
        # Un rover sin cubo propio (hay menos cubos que rovers) no lleva nada
        termina = max(fin.values()) if fin else 0.0
        costo = termina + 0.1 * sum(t1.values())
        # Recorridos que se cruzan: uno de los dos va a tener que esperar al otro
        if len(pre1) == 2:
            a, b = pre1.values()
            if _separacion(_polilinea(a), _polilinea(b)) < CROSS_GAP:
                costo += T_CRUCE
        if mejor is None or costo < mejor["costo"]:
            mejor = {"costo": round(costo, 1), "termina": round(termina, 1), "orden": orden, "rutas": listas,
                     "comodin": comodin, "primeros": pre1, "t1": t1}
    if mejor is None:
        return None

    # ¿Pueden salir los dos a la vez? Solo si ninguno empieza girando mucho o marcha atrás
    # (al pivotar barre con las pinzas) y sus primeros recorridos no se acercan.
    juntos = False
    if len(mejor["primeros"]) == 2:
        a, b = (mejor["primeros"][r] for r in sorted(mejor["primeros"]))
        def sale_derecho(rid, pre):
            pts = _polilinea(pre)
            sig = next((p for p in pts[1:] if _dist(pts[0], p) > 1.0), None)
            atras = bool(pre["ida"]) and pre["ida"][0][2]
            return sig is not None and not atras and _wrap(_heading(pts[0], sig) - poses[rid][2]) < 35.0
        r0, r1 = sorted(mejor["primeros"])
        juntos = sale_derecho(r0, a) and sale_derecho(r1, b) and _separacion(_polilinea(a), _polilinea(b)) >= CLEAR_GAP

    tareas = {rid: "".join(LETTER[c] for c in mejor["orden"][rid]) for rid in rids}
    return {
        "id": plan_id, "tareas": tareas, "rutas": mejor["rutas"], "juntos": juntos, "costo": mejor["termina"],
        "comodin": mejor["comodin"],
        "mensajes": {rid: message_completo(plan_id, tareas, mejor["rutas"][rid], juntos) for rid in rids},
        "dibujo": {rid: [{"color": pre["color"], "puntos": [[round(x, 1), round(y, 1)] for x, y in _polilinea(pre)] + [list(zonas[pre["color"]])],
                          "comodin": pre["color"] == mejor["comodin"]} for pre in mejor["rutas"][rid]] for rid in rids},
    }
