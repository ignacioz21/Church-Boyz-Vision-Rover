"""Planificador de la PC: reparte los cubos entre los rovers.

Se usa ANTES de READY (reglamento 6.2.5 y 8.6.6): con el mundo que publica la
visión decide qué cubos lleva cada rover y en qué orden, y arma el mensaje que se
carga en los dos. Las rutas no se planifican acá: cada rover las calcula a bordo.

    world = mensaje de la visión (dict)      ->      plan(world) = {10: "gb", 11: "r"}
    message(plan_id, tasks)                  ->      "P,7,10=gb,11=r"

Funciones puras: no abren sockets ni leen nada. El simulador (sim/analizar.py) las
usa tal cual para medir cuánto ayuda el plan.
"""

import itertools
import math

LETTER = {"red": "r", "green": "g", "blue": "b"}

# Modelo de tiempos (segundos). Salen del simulador con la calibración real; el
# planificador solo necesita que sean proporcionados, no exactos.
SPEED_FREE = 7.0        # celdas/s yendo sin cubo
SPEED_CARRY = 5.0       # celdas/s con un cubo en las pinzas
T_CAPTURE = 6.0         # apuntar y meter el cubo en las pinzas
T_RELEASE = 7.0         # apuntar a la zona, tramo final lento, soltar y verificar
T_CROSSING = 4.0        # castigo cuando los dos rovers pasarían por el mismo lugar a la vez
BLOCK_RADIUS = 9.0      # un cubo a menos de esto del centro de una zona la deja ocupada


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


def _timeline(start, order, cubes, depots):
    """Recorrido de un rover: [(color, t_inicio, t_toma, t_entrega, tramos)]."""
    t, pos, out = 0.0, start, []
    for color in order:
        cube, depot = cubes[color], depots[color]
        t_pick = t + _dist(pos, cube) / SPEED_FREE + T_CAPTURE
        t_drop = t_pick + _dist(cube, depot) / SPEED_CARRY + T_RELEASE
        out.append((color, t, t_pick, t_drop, [(pos, cube), (cube, depot)]))
        t, pos = t_drop, depot
    return out


def _cost(assignment, starts, cubes, depots, blockers):
    """Tiempo estimado hasta el último cubo, con castigos por estorbos."""
    lines = {rid: _timeline(starts[rid], order, cubes, depots) for rid, order in assignment.items()}
    pick = {c: t_pick for line in lines.values() for c, _, t_pick, _, _ in line}
    owner = {c: rid for rid, line in lines.items() for c, _, _, _, _ in line}

    finish, penalty = 0.0, 0.0
    for rid, line in lines.items():
        delay = 0.0
        for color, _, t_pick, t_drop, _ in line:
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
        for _, a0, _, a1, segs_a in lines[rids[0]]:
            for _, b0, _, b1, segs_b in lines[rids[1]]:
                same_time = a0 < b1 and b0 < a1
                if same_time and any(_seg_distance(*sa, *sb) < 8.0 for sa in segs_a for sb in segs_b):
                    penalty += T_CROSSING
    total = sum(line[-1][3] for line in lines.values() if line)
    return finish + penalty + 0.05 * total


def plan(world):
    """Reparto y orden de los cubos. Devuelve {id_rover: "letras en orden"}."""
    rovers = {r["id"]: (r["col"], r["row"]) for r in world.get("rovers", [])}
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
        return {}
    if not pending:
        return {rid: "" for rid in rovers}

    # Qué cubos dejan ocupada la zona de otro color
    blockers = {color: [other for other, pos in pending.items()
                        if other != color and _dist(pos, depots[color]) < BLOCK_RADIUS]
                for color in pending}

    rids = sorted(rovers)
    colors = sorted(pending)
    best, best_cost = None, float("inf")
    for owners in itertools.product(rids, repeat=len(colors)):
        groups = {rid: [c for c, o in zip(colors, owners) if o == rid] for rid in rids}
        for orders in itertools.product(*(itertools.permutations(g) for g in groups.values())):
            assignment = dict(zip(groups, orders))
            cost = _cost(assignment, rovers, pending, depots, blockers)
            if cost < best_cost:
                best, best_cost = assignment, cost
    return {rid: "".join(LETTER[c] for c in order) for rid, order in best.items()}


def message(plan_id, tasks):
    """Mensaje para los rovers (rover_*/include/plan.h): "P,<id>,10=gb,11=r"."""
    return "P,%d,%s" % (plan_id, ",".join("%d=%s" % (rid, tasks[rid]) for rid in sorted(tasks)))
