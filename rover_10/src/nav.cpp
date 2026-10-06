#include "../include/nav.h"
#include "../include/config.h"
#include "../include/geometry.h"
#include <math.h>

// --- Rejilla -------------------------------------------------------------------
static const float STEP = 2.0f;         // Celdas entre puntos de la rejilla
static const int MAX_N = 24;            // Canchas de hasta 46 celdas por lado
static const int DIRS = 8;              // Rumbos de viaje: múltiplos de 45°
static const int BINS = 16;             // Rumbos de la tabla: múltiplos de 22.5°
static const int MAX_NODES = MAX_N * MAX_N;
static const int MAX_STATES = MAX_NODES * DIRS;
static const uint16_t NONE = 0xFFFF;

// Costos en octavos de celda (caben en 16 bits)
static const uint16_t COST_STRAIGHT = 16, COST_DIAGONAL = 23;   // Avanzar a un vecino
static const uint16_t COST_REVERSE_PCT = 160;                   // Retroceder cuesta 1.6x
static const uint16_t COST_PIVOT_BIN = 10;                      // Pivotar 22.5°

// Holguras de la tabla, mayores que las de la huella real: cubren que el robot
// nunca está exactamente sobre un punto de la rejilla.
static const float TABLE_CUBE_MARGIN = 0.9f;
static const float TABLE_LINE_MARGIN = 0.5f;
static const float TABLE_PEER_RADIUS = 8.5f;
// Avanzar o retroceder RECTO es el movimiento más predecible: ahí alcanza con no
// tocar el cubo (lado 1.5 + un poco), sin la holgura que se pide para girar. Hace
// falta para salir cuando un cubo quedó pegado al costado del robot.
static const float STRAIGHT_CUBE_MARGIN = -0.4f;

static const int DX[DIRS] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static const int DY[DIRS] = { 0, -1, -1, -1, 0, 1, 1, 1 };     // row crece hacia abajo

static int NX = 0, NY = 0;
static uint16_t clear_bits[MAX_NODES];      // bit b: libre con rumbo b * 22.5°
static uint8_t repel[MAX_NODES];            // Costo extra por pasar cerca de algo (área de repelencia)
static uint16_t g_cost[MAX_STATES];
static uint16_t parent[MAX_STATES];
static uint8_t closed[MAX_STATES / 8 + 1];
static int start_node = -1;

struct __attribute__((packed)) HeapItem { uint16_t f; uint16_t state; };
static const int HEAP_CAP = MAX_STATES * 3 / 2;
static HeapItem heap[HEAP_CAP];
static int heap_n;

static void heapPush(uint16_t f, uint16_t state) {
    if (heap_n >= HEAP_CAP) return;
    int i = heap_n++;
    while (i > 0 && heap[(i - 1) / 2].f > f) {
        heap[i] = heap[(i - 1) / 2];
        i = (i - 1) / 2;
    }
    heap[i] = { f, state };
}

static HeapItem heapPop() {
    HeapItem top = heap[0];
    HeapItem last = heap[--heap_n];
    int i = 0;
    while (true) {
        int child = 2 * i + 1;
        if (child >= heap_n) break;
        if (child + 1 < heap_n && heap[child + 1].f < heap[child].f) child++;
        if (heap[child].f >= last.f) break;
        heap[i] = heap[child];
        i = child;
    }
    heap[i] = last;
    return top;
}

static inline int nodeOf(Point p) {
    int ix = constrain((int)lroundf(p.col / STEP), 0, NX - 1);
    int iy = constrain((int)lroundf(p.row / STEP), 0, NY - 1);
    return iy * NX + ix;
}
static inline Point pointOf(int node) {
    Point p = { (node % NX) * STEP, (node / NX) * STEP };
    return p;
}
static inline int dirOf(float heading) { return ((int)lroundf(heading / 45.0f) % DIRS + DIRS) % DIRS; }
static inline int binOf(float heading) { return ((int)lroundf(heading / 22.5f) % BINS + BINS) % BINS; }
static inline bool isClear(int node, int bin) { return clear_bits[node] & (1 << (bin & (BINS - 1))); }

// --- Tabla "cabe / no cabe" ------------------------------------------------------
struct Blob { Point p; float radius; float repel_radius; float repel_gain; };

static bool peer_leg_active = false;
static Point peer_leg_from, peer_leg_to;

void navSetPeerLeg(bool active, Point from, Point to) {
    peer_leg_active = active;
    peer_leg_from = from;
    peer_leg_to = to;
}

static void buildTable(const TelemetrySnapshot &snap, CubeColor carried) {
    NX = min((int)(snap.grid_cols / STEP) + 1, MAX_N);
    NY = min((int)(snap.grid_rows / STEP) + 1, MAX_N);

    static const int LEG_BLOBS = 6;
    Blob blobs[NUM_COLORS + 1 + LEG_BLOBS];
    int n = 0;
    for (int c = 0; c < NUM_COLORS; c++) {
        Point p = { snap.cubes[c].col, snap.cubes[c].row };
        if (c == carried || !snap.cubes[c].detected || !inField(p, snap)) continue;
        // Un cubo ya entregado se cuida más: moverlo anula la entrega
        bool delivered = cubeExcess(p, (CubeColor)c, snap) == 0.0f;
        blobs[n++] = { p, snap.cube_side * (float)M_SQRT2 / 2.0f + TABLE_CUBE_MARGIN + (delivered ? 0.8f : 0.0f),
                       REPEL_CUBE_RADIUS, delivered ? 28.0f : 16.0f };
    }
    // Compañero quieto: cuenta su forma real (más abajo), no el círculo grande
    bool still = snap.peer.detected && geometryPeerStill();
    PeerShape shape;
    if (still) peerShape(snap, shape);
    if (snap.peer.detected) {
        blobs[n++] = { peerCenter(snap), still ? 0.0f : TABLE_PEER_RADIUS, REPEL_PEER_RADIUS, 24.0f };
        // El tramo que el compañero va a recorrer, como una fila de círculos
        float leg = peer_leg_active ? dist(peer_leg_from, peer_leg_to) : 0.0f;
        for (int i = 1; i <= LEG_BLOBS && leg > 3.0f; i++) {
            float t = (float)i / LEG_BLOBS;
            Point p = { peer_leg_from.col + (peer_leg_to.col - peer_leg_from.col) * t,
                        peer_leg_from.row + (peer_leg_to.row - peer_leg_from.row) * t };
            blobs[n++] = { p, TABLE_PEER_RADIUS, REPEL_PEER_RADIUS, 12.0f };
        }
    }

    // Área de repelencia: costo extra (en octavos de celda) que crece al acercarse a
    // un cubo, al compañero o a una línea. No prohíbe pasar: hace que la ruta se
    // aleje cuando hay lugar y solo pase justo cuando no queda otra.
    for (int node = 0; node < NX * NY; node++) {
        Point a = pointOf(node);
        float extra = 0.0f;
        for (int i = 0; i < n; i++) {
            float d = dist(a, blobs[i].p);
            if (d < blobs[i].repel_radius) extra += blobs[i].repel_gain * (1.0f - d / blobs[i].repel_radius);
        }
        float to_line = min(min(a.col, snap.grid_cols - a.col), min(a.row, snap.grid_rows - a.row));
        if (to_line < REPEL_LINE_DIST) extra += 14.0f * (1.0f - to_line / REPEL_LINE_DIST);
        repel[node] = (uint8_t)min(extra, 250.0f);
    }

    const float tol = LINE_TOLERANCE - TABLE_LINE_MARGIN;
    const float lo = -tol, hi_c = snap.grid_cols + tol, hi_r = snap.grid_rows + tol;
    static const float CORNERS[][2] = {
        { -FP_REAR, FP_HALF_WIDTH }, { -FP_REAR, -FP_HALF_WIDTH },
        { FP_FRONT, FP_HALF_WIDTH }, { FP_FRONT, -FP_HALF_WIDTH },
        { FP_PRONG_TIP, FP_PRONG_SIDE }, { FP_PRONG_TIP, -FP_PRONG_SIDE },
    };

    for (int node = 0; node < NX * NY; node++) clear_bits[node] = 0;
    for (int bin = 0; bin < BINS; bin++) {
        float c = cosf(bin * 22.5f * (float)M_PI / 180.0f), s = sinf(bin * 22.5f * (float)M_PI / 180.0f);
        for (int node = 0; node < NX * NY; node++) {
            Point a = pointOf(node);
            bool ok = true;
            for (const auto &k : CORNERS) {
                float col = a.col + k[0] * c + k[1] * s, row = a.row - k[0] * s + k[1] * c;
                if (col < lo || col > hi_c || row < lo || row > hi_r) { ok = false; break; }
            }
            for (int i = 0; i < n && ok; i++) {
                // El obstáculo en el marco del robot (x al frente, y a un costado)
                float dx = blobs[i].p.col - a.col, dy = blobs[i].p.row - a.row;
                float x = dx * c - dy * s, y = dx * s + dy * c;
                float bx = constrain(x, -FP_REAR, FP_FRONT), by = constrain(y, -FP_HALF_WIDTH, FP_HALF_WIDTH);
                float px = constrain(x, FP_FRONT, FP_PRONG_TIP), py = y > 0 ? FP_PRONG_SIDE : -FP_PRONG_SIDE;
                float r = blobs[i].radius;
                if (hypotf(x - bx, y - by) < r || hypotf(x - px, y - py) < r) ok = false;
            }
            if (ok && still && peerShapeHits(shape, a, c, s, PEER_STILL_MARGIN + 0.5f)) ok = false;
            if (ok) clear_bits[node] |= (1 << bin);
        }
    }
}

// --- Costos desde la pose actual (Dijkstra sobre posición + rumbo) ---------------
static void relax(int from, int to, uint16_t cost) {
    if (closed[to / 8] & (1 << (to % 8))) return;
    uint32_t g = (uint32_t)g_cost[from] + cost;
    if (g < g_cost[to]) {
        g_cost[to] = (uint16_t)g;
        parent[to] = (uint16_t)from;
        heapPush((uint16_t)g, (uint16_t)to);
    }
}

static uint32_t plan_count = 0, plan_ms = 0, travel_plans = 0;
void navPlanStats(uint32_t *count, uint32_t *ms) { *count = plan_count; *ms = plan_ms; }
uint32_t navTravelPlans() { return travel_plans; }
void navPlanStatsReset() { plan_count = 0; plan_ms = 0; travel_plans = 0; }

void navPrepare(const TelemetrySnapshot &snap, const Pose &from, CubeColor carried) {
    uint32_t started_ms = millis();
    buildTable(snap, carried);
    int states = NX * NY * DIRS;
    for (int s = 0; s < states; s++) { g_cost[s] = NONE; parent[s] = NONE; }
    for (int i = 0; i <= states / 8; i++) closed[i] = 0;

    start_node = nodeOf(from.p);
    int start = start_node * DIRS + dirOf(from.theta);

    // La tabla usa holguras mayores que la huella real, así que puede dar por
    // bloqueado el lugar donde el robot YA está (p. ej. recién tomado un cubo con
    // otro a un costado). El tramo recto que sale de la pose real se evalúa con la
    // huella exacta y, donde pasa, se anota como libre.
    int d0 = dirOf(from.theta);
    for (int sign = 1; sign >= (carried == COLOR_UNKNOWN ? -1 : 1); sign -= 2) {
        for (int k = 1; k <= 6; k++) {
            int ix = start_node % NX + sign * k * DX[d0], iy = start_node / NX + sign * k * DY[d0];
            if (ix < 0 || ix >= NX || iy < 0 || iy >= NY) break;
            float along = k * STEP * ((d0 % 2) ? (float)M_SQRT2 : 1.0f);
            if (!poseClear(advance(from.p, from.theta, sign * along), from.theta, snap, carried, STRAIGHT_CUBE_MARGIN)) break;
            clear_bits[iy * NX + ix] |= (1 << (2 * d0));
        }
    }

    heap_n = 0;
    g_cost[start] = 0;
    heapPush(0, (uint16_t)start);

    // El primer giro se hace desde la pose REAL, no desde un punto de la rejilla:
    // se evalúa con la huella exacta (la tabla, con sus holguras, puede dar por
    // encerrado a un robot que en realidad sí puede girar).
    for (int nd = 0; nd < DIRS; nd++) {
        int s2 = start_node * DIRS + nd;
        if (s2 == start) continue;
        for (int turn = -1; turn <= 1; turn += 2) {
            // Con la holgura mínima: si acá el giro cabe justo, tiene que poder salir
            if (!pivotClear(from.p, from.theta, nd * 45.0f, turn, snap, carried, PIVOT_TIGHT)) continue;
            int steps = ((nd - dirOf(from.theta)) * turn % DIRS + DIRS) % DIRS;
            uint16_t cost = steps * 2 * COST_PIVOT_BIN;
            if (cost < g_cost[s2]) {
                g_cost[s2] = cost;
                parent[s2] = (uint16_t)start;
                heapPush(cost, (uint16_t)s2);
            }
        }
    }

    while (heap_n > 0) {
        HeapItem item = heapPop();
        int s = item.state;
        if (closed[s / 8] & (1 << (s % 8))) continue;
        closed[s / 8] |= (1 << (s % 8));
        int node = s / DIRS, d = s % DIRS;
        int ix = node % NX, iy = node / NX;
        // Desde donde está el robot siempre se puede salir recto, aunque la tabla
        // (con sus holguras) diga que ahí no cabe.
        bool here_ok = isClear(node, 2 * d) || node == start_node;

        for (int sign = 1; sign >= (carried == COLOR_UNKNOWN ? -1 : 1); sign -= 2) {    // Adelante y, sin cubo, atrás
            int nx = ix + sign * DX[d], ny = iy + sign * DY[d];
            if (nx < 0 || nx >= NX || ny < 0 || ny >= NY) continue;
            int nb = ny * NX + nx;
            if (!here_ok || !isClear(nb, 2 * d)) continue;
            uint16_t cost = (d % 2) ? COST_DIAGONAL : COST_STRAIGHT;
            if (sign < 0) cost = cost * COST_REVERSE_PCT / 100;
            relax(s, nb * DIRS + d, cost + repel[nb]);
        }
        for (int turn = -1; turn <= 1; turn += 2) {             // Pivotar 45°
            int nd = (d + turn + DIRS) % DIRS;
            if (!isClear(node, 2 * d) || !isClear(node, 2 * d + turn) || !isClear(node, 2 * nd)) continue;
            relax(s, node * DIRS + nd, 2 * COST_PIVOT_BIN + repel[node] / 2);
        }
    }
    plan_count++;
    plan_ms += millis() - started_ms;
}

bool navStage(Point target, int dir, int steps, Point &stage) {
    int node = nodeOf(target);
    int ix = node % NX - steps * DX[dir], iy = node / NX - steps * DY[dir];
    if (ix < 0 || ix >= NX || iy < 0 || iy >= NY) return false;
    stage = pointOf(iy * NX + ix);
    return true;
}

float navCost(Point goal, int dir) {
    if (start_node < 0) return -1.0f;
    uint16_t g = g_cost[nodeOf(goal) * DIRS + dir];
    return g == NONE ? -1.0f : g / 8.0f;
}

// --- Seguimiento --------------------------------------------------------------
static bool have_leg = false;
static Point leg_target;
static bool leg_reverse = false;
static bool leg_moving = false;         // Ya se alineó y empezó a recorrer el tramo
static Point leg_from;                  // Dónde estaba el robot al empezar el tramo
static Point planned_goal;
static int planned_dir = -1;
static uint32_t blocked_since_ms = 0;
static int failed_plans = 0;

// La ruta en curso, entera: se calcula una vez y se recorre tramo a tramo
static NavLeg legs[NAV_MAX_LEGS];
static int legs_n = 0, leg_at = 0;
static bool legs_reach_goal = false;    // false si quedó cortada (demasiado larga): al terminarla se recalcula
static bool legs_external = false;      // La cargó la PC (navSetRoute), no salió de planRoute
static bool legs_unchecked = false;     // Recién cargada: falta ver si el primer tramo cabe desde acá
static uint32_t route_ms = 0;
// Tras un cálculo sin ruta no se reintenta enseguida: cada cálculo deja al rover sordo
// medio segundo, y lo que estorba (casi siempre el compañero) no se va en un ciclo.
static uint32_t plan_retry_ms = 0;
static const uint32_t PLAN_RETRY_MS = 400;
static const uint32_t ROUTE_MAX_AGE_MS = 12000;     // Una ruta propia más vieja se recalcula en la próxima esquina
static const float OFF_ROUTE_DIST = 3.0f;           // Apartado del tramo más que esto: recalcular

int navRoute(Point *out, int max) {
    int n = 0;
    for (int i = leg_at; have_leg && i < legs_n && n < max; i++) out[n++] = legs[i].to;
    return n;
}

int navLegs(NavLeg *out, int max, bool *complete) {
    int n = min(legs_n, max);
    for (int i = 0; i < n; i++) out[i] = legs[i];
    if (complete) *complete = legs_reach_goal && n == legs_n;
    return n;
}

void navReset() {
    have_leg = false;
    legs_n = 0;
    leg_at = 0;
    legs_external = false;
    legs_unchecked = false;
    blocked_since_ms = 0;
    failed_plans = 0;
}

Point navWaypoint() { return leg_target; }
bool navHasRoute() { return have_leg; }

// Calcula la ruta y la deja en legs[] como tramos rectos (los pivotes van implícitos
// entre un tramo y el siguiente). false si no hay ruta, o si no hay que moverse (at_goal).
static bool planRoute(const Pose &pose, Point goal, int goal_dir, const TelemetrySnapshot &snap, CubeColor carried, bool &at_goal) {
    navPrepare(snap, pose, carried);
    at_goal = false;
    legs_n = 0;
    leg_at = 0;
    legs_external = false;
    legs_unchecked = false;
    int goal_state = nodeOf(goal) * DIRS + goal_dir;
    if (g_cost[goal_state] == NONE) return false;

    // La ruta se conoce del final hacia el inicio. Se guardan los últimos estados del
    // recorrido hacia atrás (los más cercanos al inicio); una ruta más larga que eso
    // queda cortada y se completa al llegar a su final.
    static const int KEEP = 64;
    static uint16_t tail[KEEP];
    int length = 0;
    for (int s = goal_state; s != NONE; s = parent[s]) length++;
    int skip = max(0, length - KEEP), n = 0;
    legs_reach_goal = skip == 0;
    for (int s = goal_state; s != NONE; s = parent[s]) {
        if (skip > 0) { skip--; continue; }
        tail[n++] = (uint16_t)s;
    }

    // tail[n-1] es el estado inicial; se avanza hacia tail[0]. Un tramo termina en un
    // pivote o al cambiar entre avanzar y retroceder.
    int end_node = -1, leg_dir = -1;
    bool reverse = false;
    for (int i = n - 1; i > 0; i--) {
        int from = tail[i], to = tail[i - 1];
        int fnode = from / DIRS, tnode = to / DIRS, d = to % DIRS;
        bool pivot = fnode == tnode;
        bool rev = !pivot && ((tnode % NX - fnode % NX) != DX[d] || (tnode / NX - fnode / NX) != DY[d]);
        if (end_node >= 0 && (pivot || d != leg_dir || rev != reverse)) {
            if (legs_n >= NAV_MAX_LEGS) { legs_reach_goal = false; end_node = -1; break; }
            legs[legs_n++] = { pointOf(end_node), reverse };
            end_node = -1;
        }
        if (pivot) continue;
        end_node = tnode;
        leg_dir = d;
        reverse = rev;
    }
    if (end_node >= 0) {
        if (legs_n < NAV_MAX_LEGS) legs[legs_n++] = { pointOf(end_node), reverse };
        else legs_reach_goal = false;
    }
    if (legs_n == 0) { at_goal = true; return false; }
    route_ms = millis();
    return true;
}

bool navPlanRoute(const Pose &pose, Point goal, int dir, const TelemetrySnapshot &snap, CubeColor carried) {
    bool at_goal;
    bool ok = planRoute(pose, goal, dir, snap, carried, at_goal);
    have_leg = false;                   // Solo se calculó: nadie la está siguiendo
    return ok || at_goal;
}

void navSetRoute(const NavLeg *in, int n, Point goal, int dir) {
    navReset();
    legs_n = min(n, NAV_MAX_LEGS);
    for (int i = 0; i < legs_n; i++) legs[i] = in[i];
    if (legs_n == 0) return;
    legs_reach_goal = n <= NAV_MAX_LEGS;
    legs_external = true;
    legs_unchecked = true;
    planned_goal = goal;
    planned_dir = dir;
    have_leg = true;
    route_ms = millis();
}

// ¿El tramo recto desde donde está el robot hasta el final del tramo cabe, con la huella
// real? Es lo que permite pasar al tramo siguiente sin volver a calcular la ruta.
static bool legClear(const Pose &pose, const NavLeg &leg, const TelemetrySnapshot &snap, CubeColor carried) {
    float run = dist(pose.p, leg.to);
    float travel = leg.reverse ? headingTo(leg.to, pose.p) : headingTo(pose.p, leg.to);
    float sign = leg.reverse ? -1.0f : 1.0f;
    for (float d = 1.0f; d <= run; d += 1.0f) {
        if (!poseClear(advance(pose.p, travel, sign * d), travel, snap, carried, STRAIGHT_CUBE_MARGIN)) return false;
    }
    return true;
}

// ¿El compañero anda cerca del tramo que sigue? La ruta se calculó esquivando dónde
// estaba y hacia dónde iba ENTONCES. Si ahora se mueve cerca, hay que volver a calcular
// (como se hacía en cada esquina); si está lejos o quieto, la ruta sigue valiendo.
static const float PEER_REPLAN_DIST = 14.0f;
static bool peerNearLeg(const Pose &pose, Point to, const TelemetrySnapshot &snap) {
    if (!snap.peer.detected) return false;
    Point pc = peerCenter(snap);
    // Quieto o en movimiento: pasar cerca de él pide una ruta recién calculada
    if (pointToSegment(pc, pose.p, to) < PEER_REPLAN_DIST) return true;
    if (geometryPeerStill()) return false;
    if (!peer_leg_active) return false;
    // Su tramo anunciado contra el mío
    float gap = min(min(pointToSegment(pose.p, peer_leg_from, peer_leg_to), pointToSegment(to, peer_leg_from, peer_leg_to)),
                    min(pointToSegment(peer_leg_from, pose.p, to), pointToSegment(peer_leg_to, pose.p, to)));
    return gap < PEER_REPLAN_DIST - 3.0f;
}

// Pone en marcha el tramo leg_at. 'check' = comprobar antes que cabe (false si no).
// 'mind_peer' = además, que el compañero no ande cerca. No se pide para el primer tramo
// de una ruta cargada por la PC: los dos arrancan lado a lado, así que siempre está
// cerca, y la PC ya calculó esa ruta con él ahí.
static bool startLeg(const Pose &pose, const TelemetrySnapshot &snap, CubeColor carried, bool check, bool mind_peer = true) {
    if (check && ((mind_peer && peerNearLeg(pose, legs[leg_at].to, snap)) || !legClear(pose, legs[leg_at], snap, carried))) return false;
    leg_target = legs[leg_at].to;
    leg_reverse = legs[leg_at].reverse;
    leg_from = pose.p;
    leg_moving = false;
    blocked_since_ms = 0;
    return true;
}

NavStatus navGo(const Pose &pose, Point goal, int goal_dir, const TelemetrySnapshot &snap, CubeColor carried) {
    bool goal_changed = dist(goal, planned_goal) > 1.0f || goal_dir != planned_dir;
    // Con un cubo y ya sobre el destino: no se persigue un punto que queda casi debajo del eje
    // (al pivotar el eje se corre un poco, y el robot daría vueltas tras él). El rumbo
    // final lo corrige quien llama, que después apunta.
    if (carried != COLOR_UNKNOWN && dist(pose.p, goal) < NAV_ARRIVE_DIST) {
        motionStop();
        have_leg = false;
        failed_plans = 0;
        return NAV_ARRIVED;
    }
    // Ruta cargada de antemano: vale si desde acá se puede entrar a su primer tramo
    if (have_leg && legs_unchecked && !goal_changed) {
        legs_unchecked = false;
        if (!startLeg(pose, snap, carried, true, false)) have_leg = false;
    }
    if (!have_leg || goal_changed) {
        motionStop();
        if (millis() < plan_retry_ms) return NAV_RUNNING;
        planned_goal = goal;
        planned_dir = goal_dir;
        bool at_goal;
        travel_plans++;
        have_leg = planRoute(pose, goal, goal_dir, snap, carried, at_goal);
        if (at_goal) { failed_plans = 0; return NAV_ARRIVED; }
        if (!have_leg) {
            plan_retry_ms = millis() + PLAN_RETRY_MS;
            return ++failed_plans > 3 ? NAV_NO_PATH : NAV_RUNNING;
        }
        startLeg(pose, snap, carried, false);
    }

    // Rumbo que debe tener el robot para recorrer el tramo (al retroceder, mira en contra)
    float d = dist(pose.p, leg_target);
    float travel = leg_reverse ? headingTo(leg_target, pose.p) : headingTo(pose.p, leg_target);
    float err = wrapDeg(travel - pose.theta);

    // Fin del tramo: llegó o se pasó por poco. Se replanifica desde donde quedó.
    // ("Se pasó" solo vale si ya venía avanzando: un tramo que empieza con un giro
    // grande tiene el objetivo "detrás" sin haberse movido.)
    if (d < 0.9f || (leg_moving && d < 2.5f && fabsf(err) > 90.0f)) {
        failed_plans = 0;
#if FOLLOW_FULL_ROUTE
        bool last = leg_at + 1 >= legs_n;
        // Fin de la ruta, sobre la meta: llegó (sin calcular otra vez para enterarse)
        if (last && legs_reach_goal && d < 0.9f) {
            motionStop();
            have_leg = false;
            return NAV_ARRIVED;
        }
        // Queda ruta: al tramo siguiente sin frenar a pensar, si sigue cabiendo
        bool current = legs_external || millis() - route_ms < ROUTE_MAX_AGE_MS;
        if (!last && current) {
            leg_at++;
            if (startLeg(pose, snap, carried, true)) return NAV_RUNNING;
        }
#endif
        motionStop();
        have_leg = false;
        return NAV_RUNNING;
    }
#if FOLLOW_FULL_ROUTE
    // Muy apartado del tramo (lo empujaron, o la pose saltó): la ruta ya no describe
    // dónde está el robot. Las correcciones chicas las hace el timón, más abajo.
    if (leg_at > 0 && pointToSegment(pose.p, leg_from, leg_target) > OFF_ROUTE_DIST) {
        motionStop();
        have_leg = false;
        return NAV_RUNNING;
    }
#endif

    static bool turning = false;
    static uint32_t turning_since_ms = 0;
    if (fabsf(err) > 25.0f || (turning && fabsf(err) > 8.0f)) {
        if (!turning) turning_since_ms = millis();
        turning = !motionTurnTo(pose, travel, 8.0f, snap, carried);
        // Un giro que no termina (no hay lado libre): se descarta el tramo
        if (turning && millis() - turning_since_ms > 6000) {
            motionStop();
            turning = false;
            have_leg = false;
            if (++failed_plans > 3) return NAV_NO_PATH;
        }
        return NAV_RUNNING;
    }
    turning = false;

    // Con la huella REAL: no meterse donde tocaría algo o saldría de la tolerancia.
    // Se espera un momento (puede ser el compañero pasando) y se replanifica.
    float sign = leg_reverse ? -1.0f : 1.0f;
    if (!poseClear(advance(pose.p, pose.theta, sign * 2.0f), pose.theta, snap, carried, STRAIGHT_CUBE_MARGIN)) {
        motionStop();
        if (blocked_since_ms == 0) blocked_since_ms = millis();
        if (millis() - blocked_since_ms > 600) {
            have_leg = false;
            blocked_since_ms = 0;
            if (++failed_plans > 3) return NAV_NO_PATH;     // La tabla insiste en una ruta que no se puede hacer
        }
        return NAV_RUNNING;
    }
    blocked_since_ms = 0;

    leg_moving = true;
    // Con cubo se frena de lejos (un frenazo lo despega de las pinzas), y cerca del
    // compañero también (hay que poder parar a tiempo); en campo libre, más tarde
    bool near_peer = snap.peer.detected && dist(pose.p, peerCenter(snap)) < PEER_SLOW_DIST;
    float brake = carried == COLOR_UNKNOWN && !near_peer ? NAV_BRAKE_GAIN : 0.04f;
    float power = constrain(POWER_MIN_MOVE + d * brake, POWER_MIN_MOVE, motionCruise());
    power = max(POWER_MIN_MOVE, power * cosf(err * (float)M_PI / 180.0f));
    float steer = constrain(err * 0.010f, -0.20f, 0.20f);
    if (leg_reverse) motionDrive(-power - steer, -power + steer);
    else motionDrive(power - steer, power + steer);
    return NAV_RUNNING;
}
