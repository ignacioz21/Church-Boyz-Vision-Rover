#include "../include/strategy.h"
#include "../include/config.h"
#include "../include/geometry.h"
#include "../include/motion.h"
#include "../include/plan.h"
#include "../include/nav.h"

// Estrategia "capturar y llevar": el rover mete el cubo por el medio del hueco de
// las pinzas y lo lleva hasta su zona con él adentro (la pinza lo arrastra también
// al girar en el sitio, despacio).
//
//   ELEGIR -> IR_A_PREPARAR -> APUNTAR -> CAPTURAR -> TRANSPORTAR -> ENTREGAR -> SOLTAR -> VERIFICAR
//
// Si ningún cubo se puede entregar (cada uno tiene su zona ocupada por otro), se
// APARTA uno a un lugar libre con el mismo recorrido, y se sigue con los demás.
//
// Todo se mide desde el EJE DE LAS RUEDAS (ver motion.h). Las rutas, con y sin
// cubo, las da nav.h: tramos rectos y giros comprobados con la huella del robot.

enum State { ST_ELEGIR, ST_IR, ST_APUNTAR, ST_CAPTURAR, ST_REAPUNTAR, ST_TRANSPORTAR, ST_ENTREGAR,
             ST_SOLTAR, ST_VERIFICAR, ST_FIN };

static const float CUBE_AHEAD = AXLE_OFFSET + CONTACT_OFFSET;   // Eje -> centro del cubo en las pinzas
static const uint32_t SELECT_PERIOD_MS = 300;                   // Elegir es caro: no en cada ciclo
static const float AIM_MAX_DEG = 12.0f;                         // Corrección de puntería admitida en un punto de preparación

// Un punto de preparación: se llega a 'stage' con rumbo dir * 45° y desde ahí se
// avanza RECTO hasta 'target' (un cubo que tomar, o la zona donde dejarlo).
struct Approach {
    Point stage;
    int dir;
    Point target;       // Dónde estaba el objetivo cuando se eligió
};

static State state = ST_ELEGIR;
static const char* state_name = "ESPERA";
static CubeColor target = COLOR_UNKNOWN;
static Approach capture;                // Cómo se toma el cubo elegido
static Approach delivery;               // Cómo se llega a dejarlo
static Point drop_target;               // Dónde se deja: su zona, o un lugar libre si se lo aparta
static bool parking = false;            // true = esta tarea solo aparta el cubo
static int parking_moves = 0;
static const int MAX_PARKING_MOVES = 4; // Por ronda, para no entrar en un ir y venir
static Point goal;
static uint32_t state_since_ms = 0;
static uint32_t last_select_ms = 0;
static int attempts[NUM_COLORS];

static uint32_t aimed_since_ms = 0;     // Desde cuándo está apuntando bien
static Point release_from;
static bool carry_completed = false;    // El cubo llegó al centro de la zona
static int verify_ok = 0;
static uint32_t verify_seq = 0;
static uint32_t yield_since_ms = 0;
static uint32_t yield_ignore_until_ms = 0;

static void enter(State s) {
    state = s;
    state_since_ms = millis();
    aimed_since_ms = 0;
}

static Point cubePos(const TelemetrySnapshot &snap, CubeColor c) {
    Point p = { snap.cubes[c].col, snap.cubes[c].row };
    return p;
}

static bool delivered(const TelemetrySnapshot &snap, CubeColor c) {
    return snap.cubes[c].detected && cubeExcess(cubePos(snap, c), c, snap) == 0.0f;
}

// Costo (celdas) de llegar a 'app.stage' y avanzar recto hasta dejar el eje a
// 'stop_dist' de app.target; negativo si no se puede. Usa el último navPrepare.
// 'carried' = cubo en las pinzas durante ese tramo recto (o el que se va a tomar).
static float approachCost(const TelemetrySnapshot &snap, const Approach &app, float stop_dist, CubeColor carried) {
    float route = navCost(app.stage, app.dir);
    if (route < 0.0f) return -1.0f;

    // El objetivo casi nunca cae justo sobre la recta de la rejilla: se apunta a él
    // desde el punto de preparación, y esa corrección tiene que ser chica.
    float aim = headingTo(app.stage, app.target);
    float run = dist(app.stage, app.target);
    if (fabsf(wrapDeg(aim - app.dir * 45.0f)) > AIM_MAX_DEG) return -1.0f;
    for (float d = run; d >= stop_dist; d -= 1.0f) {
        if (!poseClear(advance(app.target, aim, -d), aim, snap, carried)) return -1.0f;
    }
    return route + run - stop_dist;
}

// Mejor forma de llegar a dejar el cubo 'c' (que va en las pinzas) con su centro en
// 'where': la zona de acopio, o un lugar libre donde apartarlo. Usa el último
// navPrepare, que debe ser desde la pose con el cubo tomado. false si no hay.
static bool bestDrop(const TelemetrySnapshot &snap, CubeColor c, Point where, Approach &out) {
    float best = 1e9f;
    Approach app;
    app.target = where;
    for (app.dir = 0; app.dir < NAV_DIRS; app.dir++) {
        for (int steps = 4; steps <= 9; steps++) {
            if (!navStage(app.target, app.dir, steps, app.stage)) break;
            // Recorrido recto suficiente para corregir la puntería antes de soltar
            if (dist(app.stage, app.target) < CUBE_AHEAD + 4.0f) continue;
            float cost = approachCost(snap, app, CUBE_AHEAD, c);
            if (cost >= 0.0f && cost < best) {
                best = cost;
                out = app;
            }
        }
    }
    return best < 1e9f;
}

// Mejor forma de tomar el cubo 'c' desde 'pose' y llevarlo hasta 'where'; false si no
// hay. Las capturas se prueban de la más barata a la más cara, y se acepta la primera
// desde la que haya ruta hasta 'where' con el cubo en las pinzas.
static bool bestCapture(const TelemetrySnapshot &snap, const Pose &pose, CubeColor c, Point where, Approach &out) {
    static const int MAX_CANDIDATES = 40, MAX_TRIES = 5;
    static Approach list[MAX_CANDIDATES];
    static float cost[MAX_CANDIDATES];
    int n = 0;

    navPrepare(snap, pose, COLOR_UNKNOWN);
    Approach app;
    app.target = cubePos(snap, c);
    for (app.dir = 0; app.dir < NAV_DIRS; app.dir++) {
        for (int steps = 4; steps <= 8 && n < MAX_CANDIDATES; steps++) {
            if (!navStage(app.target, app.dir, steps, app.stage)) break;
            // Lo bastante atrás para que las puntas no lleguen al cubo al afinar la puntería
            if (dist(app.stage, app.target) < STAGE_DIST_MIN) continue;
            float k = approachCost(snap, app, CUBE_AHEAD, c);
            if (k >= 0.0f) { list[n] = app; cost[n] = k; n++; }
        }
    }

    for (int tries = 0; tries < MAX_TRIES; tries++) {
        int i = -1;
        for (int j = 0; j < n; j++) {
            if (cost[j] >= 0.0f && (i < 0 || cost[j] < cost[i])) i = j;
        }
        if (i < 0) return false;
        cost[i] = -1.0f;                // Ya probada

        // Con el cubo tomado: ¿hay ruta hasta su zona?
        Pose captured;
        captured.theta = headingTo(list[i].stage, list[i].target);
        captured.p = advance(list[i].target, captured.theta, -CUBE_AHEAD);
        navPrepare(snap, captured, c);
        Approach unused;
        if (bestDrop(snap, c, where, unused)) {
            out = list[i];
            return true;
        }
    }
    return false;
}

// ¿Es 'p' un buen lugar para apartar el cubo 'c'? Lejos de las zonas (para no
// estorbar entregas), de los demás cubos y de donde está ahora.
static bool goodParking(const TelemetrySnapshot &snap, CubeColor c, Point p) {
    if (dist(p, cubePos(snap, c)) < 8.0f) return false;
    for (int o = 0; o < NUM_COLORS; o++) {
        if (dist(p, snap.depots[o]) < 13.0f) return false;
        if (o != c && snap.cubes[o].detected && dist(p, cubePos(snap, (CubeColor)o)) < 10.0f) return false;
    }
    return true;
}

// Siguiente tarea, en el orden del plan:
//  1. Un cubo que se pueda tomar y llevar a su zona ahora.
//  2. Si ninguno se puede (p. ej. cada cubo tiene su zona ocupada por otro): APARTAR
//     uno a un lugar libre, para destrabar la entrega de los demás.
static CubeColor selectTask(const TelemetrySnapshot &snap, const Pose &pose) {
    const Plan &plan = planGet(snap);
    bool pending[NUM_COLORS] = { false, false, false };
    for (int i = 0; i < plan.n_mine; i++) {
        CubeColor c = plan.mine[i];
        pending[c] = snap.cubes[c].detected && inField(cubePos(snap, c), snap) &&
                     !delivered(snap, c) && attempts[c] < MAX_ATTEMPTS;
    }

    for (int i = 0; i < plan.n_mine; i++) {
        CubeColor c = plan.mine[i];
        if (!pending[c]) continue;
        if (bestCapture(snap, pose, c, snap.depots[c], capture)) {
            drop_target = snap.depots[c];
            parking = false;
            return c;
        }
    }

    if (parking_moves >= MAX_PARKING_MOVES) return COLOR_UNKNOWN;
    // Lugares candidatos: una grilla gruesa por el interior de la cancha
    for (int i = 0; i < plan.n_mine; i++) {
        CubeColor c = plan.mine[i];
        if (!pending[c]) continue;
        for (float col = 0.25f; col <= 0.76f; col += 0.25f) {
            for (float row = 0.25f; row <= 0.76f; row += 0.25f) {
                Point p = { snap.grid_cols * col, snap.grid_rows * row };
                if (!goodParking(snap, c, p)) continue;
                if (bestCapture(snap, pose, c, p, capture)) {
                    drop_target = p;
                    parking = true;
                    parking_moves++;
                    return c;
                }
            }
        }
    }
    return COLOR_UNKNOWN;
}

// El rover de ID mayor cede si el compañero está cerca y en su camino
static bool mustYield(const Pose &pose, Point wp, const TelemetrySnapshot &snap) {
    if (ROVER_ID < ROVER_PEER_ID || !snap.peer.detected) return false;
    if (millis() < yield_ignore_until_ms) return false;
    Point peer = { snap.peer.col, snap.peer.row };
    bool in_my_way = dist(pose.p, peer) < YIELD_DIST &&
                     fabsf(wrapDeg(headingTo(pose.p, peer) - headingTo(pose.p, wp))) < 60.0f;
    if (!in_my_way) {
        yield_since_ms = 0;
        return false;
    }
    if (yield_since_ms == 0) yield_since_ms = millis();
    if (millis() - yield_since_ms > YIELD_MAX_MS) {     // Ya esperó bastante: rodear
        yield_since_ms = 0;
        yield_ignore_until_ms = millis() + YIELD_MAX_MS;
        return false;
    }
    return true;
}

// Suelta lo que lleve y retrocede; 'completed' = el cubo quedó donde debía
static void release(const Pose &pose, bool completed) {
    motionStop();
    carry_completed = completed;
    if (!completed) attempts[target]++;
    release_from = pose.p;
    enter(ST_SOLTAR);
}

// Apunta hacia 'to' y devuelve true cuando lleva así, quieto, el tiempo suficiente
// para que la cámara lo confirme (la pose predicha sola no alcanza).
static bool aimAt(const Pose &pose, Point to, const TelemetrySnapshot &snap, CubeColor carried) {
    if (!motionTurnTo(pose, headingTo(pose.p, to), 4.0f, snap, carried)) aimed_since_ms = 0;
    else if (aimed_since_ms == 0) aimed_since_ms = millis();
    else if (millis() - aimed_since_ms > motionCal().latency_ms + 100) return true;
    return false;
}

void strategyReset() {
    motionStop();
    planNewRound();
    navReset();
    target = COLOR_UNKNOWN;
    state_name = "ESPERA";
    for (int i = 0; i < NUM_COLORS; i++) attempts[i] = 0;
    parking_moves = 0;
    yield_since_ms = 0;
    yield_ignore_until_ms = 0;
    last_select_ms = 0;
    enter(ST_ELEGIR);
}

void strategyStep(const TelemetrySnapshot &snap) {
    Pose pose = motionPredict(snap);

    if (state == ST_ELEGIR || state == ST_FIN) {
        motionStop();
        if (millis() - last_select_ms < SELECT_PERIOD_MS && last_select_ms != 0) return;
        last_select_ms = millis();
        target = selectTask(snap, pose);
        if (target == COLOR_UNKNOWN) {
            // Sin tareas posibles: quieto, pero se sigue mirando (un cubo puede
            // salir de su zona, o destaparse el que estaba tapado)
            state_name = "FIN";
            goal = pose.p;
            state = ST_FIN;
            return;
        }
        navReset();
        enter(ST_IR);
    }

    Point cube = cubePos(snap, target);
    Point depot = snap.depots[target];

    // Dónde queda el cubo respecto al robot (por delante del eje / hacia un costado), de dos formas:
    //  - "visto": rover y cubo del MISMO cuadro de cámara. Exacto aunque los dos se
    //    muevan juntos (cubo en las pinzas), pero con medio segundo de atraso.
    //  - "ahora": con la pose predicha. Vale mientras el cubo esté quieto (antes de tocarlo).
    bool cube_fresh = snap.cubes[target].age_ms < 400;
    Point me_seen = { snap.me.col, snap.me.row };
    Point axle_seen = advance(me_seen, snap.me.theta, -AXLE_OFFSET);
    float bearing_seen = wrapDeg(headingTo(axle_seen, cube) - snap.me.theta) * (float)M_PI / 180.0f;
    float ahead_seen = dist(axle_seen, cube) * cosf(bearing_seen);
    float aside_seen = dist(axle_seen, cube) * sinf(bearing_seen);
    float bearing_now = wrapDeg(headingTo(pose.p, cube) - pose.theta) * (float)M_PI / 180.0f;
    float ahead_now = dist(pose.p, cube) * cosf(bearing_now);
    float aside_now = dist(pose.p, cube) * sinf(bearing_now);

    // Con el cubo en las pinzas: se lo estima desde la pose predicha, a la separación
    // que mide la cámara (o la nominal si el rover lo tapa).
    float gap = cube_fresh ? constrain(ahead_seen, CUBE_AHEAD - 1.0f, CUBE_AHEAD + 1.0f) : CUBE_AHEAD;
    Point cube_carried = advance(pose.p, pose.theta, gap);
    bool cube_lost = cube_fresh && millis() - state_since_ms > 1500 &&
                     (ahead_seen > CUBE_AHEAD + 2.5f || fabsf(aside_seen) > 2.5f);

    switch (state) {
    case ST_IR: {
        state_name = "IR_A_PREPARAR";
        // Ya está entregado, o se movió y la captura elegida ya no sirve: elegir de nuevo
        if (delivered(snap, target) || dist(cube, capture.target) > 1.5f) { enter(ST_ELEGIR); break; }
        NavStatus nav = navGo(pose, capture.stage, capture.dir, snap, COLOR_UNKNOWN);
        goal = navWaypoint();
        if (nav == NAV_NO_PATH) { attempts[target]++; enter(ST_ELEGIR); break; }
        if (nav == NAV_ARRIVED) { enter(ST_APUNTAR); break; }
        if (mustYield(pose, goal, snap)) {
            state_name = "CEDER";
            motionStop();
        }
        break;
    }

    case ST_APUNTAR:
        state_name = "APUNTAR";
        goal = cube;
        if (dist(cube, capture.target) > 1.5f) { enter(ST_ELEGIR); break; }    // El cubo se movió
        // No logra apuntar (no hay lado libre para girar): probar otra captura
        if (millis() - state_since_ms > 8000) { attempts[target]++; enter(ST_ELEGIR); break; }
        if (aimAt(pose, cube, snap, COLOR_UNKNOWN)) enter(ST_CAPTURAR);
        break;

    case ST_CAPTURAR: {
        state_name = "CAPTURAR";
        goal = cube;
        if (millis() - state_since_ms > 10000) { release(pose, false); break; }

        // Ya está contra el frente: a llevarlo
        if (ahead_now < CUBE_AHEAD + 0.6f) {
            // Entrada a la zona, con el cubo ya en las pinzas
            navPrepare(snap, pose, target);
            if (!bestDrop(snap, target, drop_target, delivery)) { release(pose, false); break; }
            navReset();
            enter(ST_TRANSPORTAR);
            break;
        }

        // Antes de que las puntas lleguen al cubo tiene que venir centrado (entra
        // con ±1.25 celdas): si no, una punta lo golpea.
        if (ahead_now < FP_PRONG_TIP + 3.0f && fabsf(aside_now) > 0.9f) {
            motionStop();
            enter(ST_REAPUNTAR);
            break;
        }

        // Recto y lento hacia el centro del cubo
        float diff = wrapDeg(headingTo(pose.p, cube) - pose.theta);
        float power = POWER_MIN_MOVE + 0.05f;
        float steer = constrain(diff * 0.012f, -0.12f, 0.12f);
        motionDrive(power - steer, power + steer);
        break;
    }

    case ST_REAPUNTAR: {
        // Retrocede recto hasta tener el cubo bien por delante de las puntas y vuelve a apuntar
        state_name = "REAPUNTAR";
        goal = cube;
        bool far_enough = dist(pose.p, cube) > FP_PRONG_TIP + 4.0f;
        if (far_enough || millis() - state_since_ms > 2500) {
            if (aimAt(pose, cube, snap, COLOR_UNKNOWN)) enter(ST_CAPTURAR);
        } else {
            motionDrive(-0.28f, -0.28f);
        }
        break;
    }

    case ST_TRANSPORTAR: {
        state_name = "TRANSPORTAR";
        if (cube_lost) { release(pose, false); break; }
        NavStatus nav = navGo(pose, delivery.stage, delivery.dir, snap, target);
        goal = navWaypoint();
        if (nav == NAV_NO_PATH) { release(pose, false); break; }
        if (nav == NAV_ARRIVED) enter(ST_ENTREGAR);
        break;
    }

    case ST_ENTREGAR: {
        state_name = parking ? "APARTAR" : "ENTREGAR";
        Point drop = drop_target;
        goal = drop;
        // Primero apunta al destino (girando con el cubo), después avanza recto
        static bool aimed = false;
        if (millis() - state_since_ms < 40) aimed = false;
        if (!aimed) {
            aimed = aimAt(pose, drop, snap, target) || millis() - state_since_ms > 8000;
            break;
        }

        // Llegó, o se pasó (el cubo ya no queda entre el rover y el destino)
        bool passed = (drop.col - cube_carried.col) * (drop.col - pose.p.col) +
                      (drop.row - cube_carried.row) * (drop.row - pose.p.row) < 0.0f;
        if (dist(cube_carried, drop) < 1.0f || passed) { release(pose, true); break; }

        bool blocked = !poseClear(advance(pose.p, pose.theta, 1.5f), pose.theta, snap, target, 0.0f);
        bool timed_out = millis() - state_since_ms > PUSH_TIMEOUT_MS;
        if (cube_lost || blocked || timed_out) { release(pose, false); break; }

        float left, right;
        carryCommand(pose, drop, gap, left, right);
        motionDrive(left, right);
        break;
    }

    case ST_SOLTAR: {
        state_name = "SOLTAR";
        goal = release_from;
        // Retroceso recto; se corta si la cola va a tocar algo o salirse
        bool rear_clear = poseClear(advance(pose.p, pose.theta, -1.5f), pose.theta, snap, target, 0.0f);
        if (dist(pose.p, release_from) >= RETREAT_DIST || !rear_clear || millis() - state_since_ms > 2500) {
            motionStop();
            verify_ok = 0;
            verify_seq = snap.seq;
            // Solo una entrega se verifica; tras apartar un cubo se vuelve a elegir
            enter(carry_completed && !parking ? ST_VERIFICAR : ST_ELEGIR);
        } else {
            motionDrive(-0.30f, -0.30f);
        }
        break;
    }

    case ST_VERIFICAR:
        state_name = "VERIFICAR";
        goal = depot;
        motionStop();
        // Solo cuentan cuadros nuevos en los que la cámara ve el cubo de verdad
        if (snap.seq != verify_seq && snap.cubes[target].age_ms < 300) {
            verify_seq = snap.seq;
            verify_ok = delivered(snap, target) ? verify_ok + 1 : 0;
        }
        if (verify_ok >= 5) {
            enter(ST_ELEGIR);                           // Entregado
        } else if (millis() - state_since_ms > 2500) {
            attempts[target]++;                         // No quedó dentro: reintentar o pasar al siguiente
            enter(ST_ELEGIR);
        }
        break;

    default:
        break;
    }
}

const char* strategyStateName() { return state_name; }
CubeColor strategyTarget() { return target; }
Point strategyGoal() { return goal; }
