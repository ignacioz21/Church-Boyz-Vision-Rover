#include "../include/strategy.h"
#include "../include/config.h"
#include "../include/geometry.h"
#include "../include/motion.h"
#include "../include/plan.h"
#include "../include/nav.h"
#include "../include/coord.h"

// Estrategia "capturar y llevar": el rover mete el cubo por el medio del hueco de
// las pinzas y lo lleva hasta su zona con él adentro (la pinza lo arrastra también
// al girar en el sitio, despacio).
//
//   ELEGIR -> IR_A_PREPARAR -> APUNTAR -> CAPTURAR -> TRANSPORTAR -> ENTREGAR -> SOLTAR -> VERIFICAR
//
// Todo se mide desde el EJE DE LAS RUEDAS (ver motion.h). Las rutas, con y sin
// cubo, las da nav.h: tramos rectos y giros comprobados con la huella del robot.
//
// Con el compañero (coord.h) las reglas son:
//   - Pasa primero el que lleva un cubo; si no, el de menor ID. El otro espera.
//   - Si lo que me traba lo puede resolver el compañero, ESPERO en vez de apartar cubos.
//   - Si no tengo nada que hacer y estoy en su camino, me corro (DESPEJAR).
//   - Si terminé lo mío, AYUDO con los cubos que él todavía no está atendiendo.
// Si los mensajes no llegan, todo sigue funcionando con la cámara y el plan.

enum State { ST_ELEGIR, ST_IR, ST_APUNTAR, ST_CAPTURAR, ST_REAPUNTAR, ST_TRANSPORTAR, ST_ENTREGAR,
             ST_SOLTAR, ST_VERIFICAR, ST_DESPEJAR, ST_FIN };

static const float CUBE_AHEAD = AXLE_OFFSET + CONTACT_OFFSET;   // Eje -> centro del cubo en las pinzas
static const uint32_t SELECT_PERIOD_MS = 300;                   // Elegir es caro: no en cada ciclo
static const float AIM_MAX_DEG = 12.0f;                         // Corrección de puntería admitida en un punto de preparación
static const uint32_t GIVE_UP_MS = 45000;                       // Esperando sin poder hacer nada: se da por terminado

// Un punto de preparación: se llega a 'stage' con rumbo dir * 45° y desde ahí se
// avanza RECTO hasta 'target' (un cubo que tomar, o el lugar donde dejarlo).
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
static bool have_delivery = false;
static bool holding = false;            // El tramo en curso va a un punto intermedio, no al destino
static Point hold_point;
static bool hold_used = false;          // Ya se hizo esa maniobra con este cubo
static Approach rest;                   // Adónde me corro para no estorbar
static Point drop_target;               // Dónde se deja: su zona, o un lugar libre si se lo aparta
static bool parking = false;            // true = esta tarea solo aparta el cubo
static int parking_moves = 0;
static const int MAX_PARKING_MOVES = 4; // Por ronda, para no entrar en un ir y venir
static Point goal;
static uint32_t state_since_ms = 0;
static uint32_t last_select_ms = 0;
static int attempts[NUM_COLORS];
static StrategyStats stats;
static const char* why[NUM_COLORS] = { "-", "-", "-" };

// Formas de tomar un cubo que ya fallaron: por un rato no se vuelven a elegir, así
// el siguiente intento prueba algo distinto en vez de repetir lo mismo.
struct Banned { CubeColor cube; Point stage; int dir; uint32_t until_ms; };
static const int MAX_BANNED = 10;
static Banned banned[MAX_BANNED];
static int banned_next = 0;
static const uint32_t BAN_MS = 40000;

// Vigilante de progreso: desde dónde y desde cuándo no avanza
static Point progress_from;
static float progress_theta = 0.0f;
static uint32_t progress_since_ms = 0;

static uint32_t aimed_since_ms = 0;     // Desde cuándo está apuntando bien
static uint32_t idle_since_ms = 0;      // Desde cuándo no tiene nada que pueda hacer
static uint32_t stuck_since_ms = 0;     // Desde cuándo no encuentra ruta (con el compañero cerca)
static uint32_t cleared_until_ms = 0;   // Hasta cuándo no vuelve a correrse
static bool all_mine_done = false;
static Point release_from;
static bool carry_completed = false;    // El cubo llegó a su destino
static int verify_ok = 0;
static uint32_t verify_seq = 0;
static uint32_t yield_since_ms = 0;
static uint32_t yield_ignore_until_ms = 0;

static void enter(State s) {
    state = s;
    state_since_ms = millis();
    aimed_since_ms = 0;
    stuck_since_ms = 0;
}

static Point cubePos(const TelemetrySnapshot &snap, CubeColor c) {
    Point p = { snap.cubes[c].col, snap.cubes[c].row };
    return p;
}

static bool delivered(const TelemetrySnapshot &snap, CubeColor c) {
    return snap.cubes[c].detected && cubeExcess(cubePos(snap, c), c, snap) == 0.0f;
}

// Un cubo que está en la cancha y todavía no quedó en su zona
static bool pendingCube(const TelemetrySnapshot &snap, CubeColor c) {
    return snap.cubes[c].detected && inField(cubePos(snap, c), snap) && !delivered(snap, c);
}

static void ban(CubeColor c, const Approach &a) {
    banned[banned_next] = { c, a.stage, a.dir, millis() + BAN_MS };
    banned_next = (banned_next + 1) % MAX_BANNED;
}

static bool isBanned(CubeColor c, const Approach &a) {
    for (const Banned &b : banned) {
        if (b.cube == c && b.dir == a.dir && millis() < b.until_ms && dist(b.stage, a.stage) < 1.0f) return true;
    }
    return false;
}

static Point peerMarker(const TelemetrySnapshot &snap) {
    Point p = { snap.peer.col, snap.peer.row };
    return p;
}

static bool peerIsMoving(const PeerState &peer) {
    return peer.heard && peer.activity != ACT_IDLE && peer.activity != ACT_DONE;
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

// El mundo como quedará cuando el compañero se lleve lo suyo: sin el cubo que dice
// estar atendiendo y sin el primero que le queda en el plan. Sirve para no esperar
// de más: si su cubo ocupa mi zona, igual puedo ir tomando el mío.
static void withoutPeerCubes(const TelemetrySnapshot &snap, TelemetrySnapshot &out) {
    out = snap;
    const PeerState &peer = coordPeer();
    if (!peer.heard || peer.activity == ACT_DONE) return;
    for (int c = 0; c < NUM_COLORS; c++) {
        if (coordPeerClaims((CubeColor)c)) out.cubes[c].detected = false;
    }
    const Plan &plan = planGet(snap);
    for (int i = 0; i < plan.n_peer; i++) {
        if (!pendingCube(snap, plan.peer[i])) continue;
        out.cubes[plan.peer[i]].detected = false;
        break;
    }
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
            if (isBanned(c, app)) continue;             // Ya falló hace poco: probar otra
            float k = approachCost(snap, app, CUBE_AHEAD, c);
            if (k >= 0.0f) { list[n] = app; cost[n] = k; n++; }
        }
    }

    static TelemetrySnapshot later;     // El mundo sin lo que el compañero se va a llevar
    withoutPeerCubes(snap, later);

    for (int tries = 0; tries < MAX_TRIES; tries++) {
        int i = -1;
        for (int j = 0; j < n; j++) {
            if (cost[j] >= 0.0f && (i < 0 || cost[j] < cost[i])) i = j;
        }
        if (i < 0) {
            why[c] = n == 0 ? "sin forma de tomarlo" : "sin ruta hasta el destino";
            return false;
        }
        cost[i] = -1.0f;                // Ya probada

        // Con el cubo tomado: ¿hay ruta hasta su destino?
        Pose captured;
        captured.theta = headingTo(list[i].stage, list[i].target);
        captured.p = advance(list[i].target, captured.theta, -CUBE_AHEAD);
        navPrepare(later, captured, c);
        Approach unused;
        if (bestDrop(later, c, where, unused)) {
            out = list[i];
            return true;
        }
    }
    why[c] = "sin ruta hasta el destino";
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

// ¿Es mal lugar para dejar un cubo? Pegado a otro cubo, frente a una zona o junto al
// borde: después nadie podría tomarlo.
static bool badSpot(const TelemetrySnapshot &snap, CubeColor c, Point p) {
    if (min(min(p.col, snap.grid_cols - p.col), min(p.row, snap.grid_rows - p.row)) < 7.0f) return true;
    for (int o = 0; o < NUM_COLORS; o++) {
        if (dist(p, snap.depots[o]) < 12.0f) return true;
        if (o != c && snap.cubes[o].detected && dist(p, cubePos(snap, (CubeColor)o)) < 10.0f) return true;
    }
    return false;
}

// Con el cubo en las pinzas y sin poder entregarlo: en vez de soltarlo donde estorbe,
// llevarlo a un lugar despejado. Deja armado ese transporte y devuelve true.
static bool carryToSafety(const TelemetrySnapshot &snap, const Pose &pose, CubeColor c, Point cube_now) {
    if (parking || parking_moves >= MAX_PARKING_MOVES || !badSpot(snap, c, cube_now)) return false;
    navPrepare(snap, pose, c);
    for (float col = 0.25f; col <= 0.76f; col += 0.25f) {
        for (float row = 0.25f; row <= 0.76f; row += 0.25f) {
            Point p = { snap.grid_cols * col, snap.grid_rows * row };
            bool free_spot = dist(p, cube_now) > 4.0f && !badSpot(snap, c, p);
            if (!free_spot || !bestDrop(snap, c, p, delivery)) continue;
            drop_target = p;
            parking = true;
            parking_moves++;
            stats.parks++;
            have_delivery = true;
            return true;
        }
    }
    return false;
}

// Llevo un cubo y mi destino está ocupado por el compañero, que también lleva uno
// (típico: cada cubo estaba en la zona del otro). Si yo no tengo prioridad, salgo de
// ahí con el cubo hacia un punto intermedio despejado: así su zona queda libre, él
// entrega y me libera la mía. Deja armado ese tramo y devuelve true.
static bool moveToHold(const TelemetrySnapshot &snap, const Pose &pose, CubeColor c) {
    const PeerState &peer = coordPeer();
    if (hold_used || !peer.heard || !peer.carrying || coordIHavePriority(true)) return false;
    navPrepare(snap, pose, c);
    float best = 1e9f;
    Approach app;
    for (float col = 0.2f; col <= 0.81f; col += 0.15f) {
        for (float row = 0.2f; row <= 0.81f; row += 0.15f) {
            Point p = { snap.grid_cols * col, snap.grid_rows * row };
            // Lo que importa: fuera del destino del compañero y sin quedar pegado a él
            bool clear = dist(p, peer.goal) > 15.0f && dist(p, peerCenter(snap)) > 12.0f;
            for (int o = 0; o < NUM_COLORS && clear; o++) {
                clear = dist(p, snap.depots[o]) > 11.0f &&
                        (o == c || o == peer.cube || !snap.cubes[o].detected || dist(p, cubePos(snap, (CubeColor)o)) > 9.0f);
            }
            if (!clear || !bestDrop(snap, c, p, app)) continue;
            float cost = navCost(app.stage, app.dir);
            if (cost < best) { best = cost; delivery = app; hold_point = p; }
        }
    }
    if (best >= 1e9f) return false;
    hold_used = true;
    holding = true;
    have_delivery = true;
    return true;
}

// Apartar un cubo desordena la cancha: solo cuando esperar no va a resolver nada.
static bool mayPark(const TelemetrySnapshot &snap, const Plan &plan) {
    if (parking_moves >= MAX_PARKING_MOVES) return false;
    const PeerState &peer = coordPeer();
    uint32_t idle_ms = idle_since_ms ? millis() - idle_since_ms : 0;
    if (peer.heard) {
        if (peer.activity == ACT_DONE) return true;                     // Nadie más va a mover nada
        if (peerIsMoving(peer)) return idle_ms > 2 * PEER_PATIENCE_MS;  // Está trabajando: darle tiempo
        // Los dos esperando: desempata el de menor ID; el otro le da un rato
        return ROVER_ID < ROVER_PEER_ID || idle_ms > PEER_PATIENCE_MS;
    }
    // Sin noticias suyas: si está en la cancha y le quedan cubos, darle tiempo
    bool peer_has_work = false;
    for (int i = 0; i < plan.n_peer; i++) peer_has_work = peer_has_work || pendingCube(snap, plan.peer[i]);
    if (snap.peer.detected && peer_has_work) return idle_ms > PEER_PATIENCE_MS;
    return true;
}

// Siguiente tarea:
//  1. Un cubo MÍO (en el orden del plan) que se pueda tomar y llevar a su zona ahora.
//  2. AYUDAR: un cubo del compañero que él todavía no esté atendiendo.
//  3. Si nada se puede entregar y esperar no lo va a resolver: APARTAR un cubo.
static CubeColor selectTask(const TelemetrySnapshot &snap, const Pose &pose) {
    const Plan &plan = planGet(snap);
    const PeerState &peer = coordPeer();

    all_mine_done = true;
    for (int i = 0; i < plan.n_mine; i++) {
        CubeColor c = plan.mine[i];
        bool pending = pendingCube(snap, c);
        if (pending && attempts[c] < MAX_ATTEMPTS) all_mine_done = false;
        why[c] = !snap.cubes[c].detected || !inField(cubePos(snap, c), snap) ? "no se ve en la cancha" :
                 !pending ? "entregado" :
                 attempts[c] >= MAX_ATTEMPTS ? "intentos agotados" : "en cola";
    }

    for (int i = 0; i < plan.n_mine; i++) {
        CubeColor c = plan.mine[i];
        if (!pendingCube(snap, c) || attempts[c] >= MAX_ATTEMPTS) continue;
        if (coordPeerClaims(c)) { why[c] = "lo atiende el compañero"; continue; }
        if (bestCapture(snap, pose, c, snap.depots[c], capture)) {
            drop_target = snap.depots[c];
            parking = false;
            why[c] = "en curso";
            return c;
        }
    }

    // Ayudar solo si nos estamos escuchando: si no, podríamos ir los dos por el mismo
    // cubo. Se empieza por el ÚLTIMO de su lista, que es al que él llegará más tarde.
    if (peer.heard) {
        for (int i = plan.n_peer - 1; i >= 0; i--) {
            CubeColor c = plan.peer[i];
            if (!pendingCube(snap, c) || attempts[c] >= MAX_ATTEMPTS || coordPeerClaims(c)) continue;
            if (bestCapture(snap, pose, c, snap.depots[c], capture)) {
                drop_target = snap.depots[c];
                parking = false;
                stats.helps++;
                return c;
            }
        }
    }

    if (!mayPark(snap, plan)) return COLOR_UNKNOWN;
    // Lugares candidatos: una grilla gruesa por el interior de la cancha
    for (int i = 0; i < plan.n_mine; i++) {
        CubeColor c = plan.mine[i];
        if (!pendingCube(snap, c) || attempts[c] >= MAX_ATTEMPTS || coordPeerClaims(c)) continue;
        for (float col = 0.25f; col <= 0.76f; col += 0.25f) {
            for (float row = 0.25f; row <= 0.76f; row += 0.25f) {
                Point p = { snap.grid_cols * col, snap.grid_rows * row };
                if (!goodParking(snap, c, p)) continue;
                if (bestCapture(snap, pose, c, p, capture)) {
                    drop_target = p;
                    parking = true;
                    parking_moves++;
                    stats.parks++;
                    why[c] = "apartando";
                    return c;
                }
            }
        }
    }
    return COLOR_UNKNOWN;
}

// --- No estorbar ------------------------------------------------------------------

// Sin nada que hacer: ¿estoy parado donde el compañero necesita pasar o trabajar?
static bool inTheWay(const TelemetrySnapshot &snap, const Pose &pose) {
    if (!snap.peer.detected) return false;
    const PeerState &peer = coordPeer();
    // El compañero avisa que no encuentra por dónde pasar, y estoy cerca de él o de su meta
    if (peer.heard && peer.blocked &&
        (dist(pose.p, peerMarker(snap)) < 22.0f || dist(pose.p, peer.goal) < 22.0f)) return true;
    if (peerIsMoving(peer)) {
        if (pointToSegment(pose.p, peerMarker(snap), peer.goal) < 11.0f) return true;
        if (peer.cube != COLOR_UNKNOWN && dist(pose.p, snap.depots[peer.cube]) < 13.0f) return true;
    }
    for (int c = 0; c < NUM_COLORS; c++) {
        if (!pendingCube(snap, (CubeColor)c)) continue;
        if (dist(pose.p, snap.depots[c]) < 13.0f || dist(pose.p, cubePos(snap, (CubeColor)c)) < 11.0f) return true;
    }
    return false;
}

// En tiempo real: ¿estoy parado sobre el tramo que el compañero está recorriendo
// ahora, y él tiene prioridad? Entonces no alcanza con frenar: hay que correrse.
static bool onPeerPath(const TelemetrySnapshot &snap, const Pose &pose) {
    const PeerState &peer = coordPeer();
    if (!peerIsMoving(peer) || coordIHavePriority(false)) return false;
    Point pm = peerMarker(snap);
    return dist(pose.p, pm) < 24.0f && pointToSegment(pose.p, pm, peer.waypoint) < 8.0f;
}

// Qué tan despejado queda 'p' (distancia a lo más cercano que importa)
static float clearance(const TelemetrySnapshot &snap, Point p) {
    const PeerState &peer = coordPeer();
    float best = dist(p, peerCenter(snap)) - 13.0f;
    if (peerIsMoving(peer)) {
        best = min(best, pointToSegment(p, peerMarker(snap), peer.goal) - 12.0f);
        best = min(best, dist(p, peer.goal) - 16.0f);
        if (peer.cube != COLOR_UNKNOWN) best = min(best, dist(p, snap.depots[peer.cube]) - 16.0f);
    }
    for (int c = 0; c < NUM_COLORS; c++) {
        if (!snap.cubes[c].detected) continue;
        bool pending = pendingCube(snap, (CubeColor)c);
        best = min(best, dist(p, cubePos(snap, (CubeColor)c)) - (pending ? 12.0f : 7.0f));
        if (pending) best = min(best, dist(p, snap.depots[c]) - 14.0f);
    }
    return best;        // >= 0: no estorba
}

// Lugar al que correrse: el más cercano que no estorbe (o el menos malo)
static bool findRest(const TelemetrySnapshot &snap, const Pose &pose, Approach &out) {
    navPrepare(snap, pose, COLOR_UNKNOWN);
    float best_cost = 1e9f, best_clear = -1e9f;
    bool found_clear = false;
    for (float col = 6.5f; col <= snap.grid_cols - 6.0f; col += 7.5f) {
        for (float row = 6.5f; row <= snap.grid_rows - 6.0f; row += 7.5f) {
            Point p = { col, row };
            float clear = clearance(snap, p);
            for (int d = 0; d < NAV_DIRS; d++) {
                float cost = navCost(p, d);
                if (cost < 0.0f) continue;
                bool better = clear >= 0.0f ? (!found_clear || cost < best_cost)
                                            : (!found_clear && clear > best_clear);
                if (!better) continue;
                found_clear = found_clear || clear >= 0.0f;
                best_cost = cost;
                best_clear = clear;
                out.stage = p;
                out.dir = d;
                out.target = p;
            }
        }
    }
    return best_cost < 1e9f && dist(out.stage, pose.p) > 3.0f;
}

// --- Ceder el paso ------------------------------------------------------------------

// ¿Tengo que detenerme para que pase el compañero? Solo si él tiene prioridad, se
// está moviendo y nuestros caminos se cruzan.
static bool mustYield(const Pose &pose, Point wp, const TelemetrySnapshot &snap, bool i_carry) {
    if (!snap.peer.detected || coordIHavePriority(i_carry)) return false;
    const PeerState &peer = coordPeer();
    // Un compañero quieto es un obstáculo: se lo rodea, no se lo espera
    if (peer.heard && !peerIsMoving(peer)) return false;
    if (millis() < yield_ignore_until_ms) return false;

    Point pm = peerMarker(snap);
    bool in_my_way = dist(pose.p, pm) < YIELD_DIST &&
                     fabsf(wrapDeg(headingTo(pose.p, pm) - headingTo(pose.p, wp))) < 60.0f;
    if (peer.heard) {
        // Con sus mensajes: se comparan los dos tramos, no solo las posiciones
        float gap = min(min(pointToSegment(pose.p, pm, peer.waypoint), pointToSegment(wp, pm, peer.waypoint)),
                        min(pointToSegment(pm, pose.p, wp), pointToSegment(peer.waypoint, pose.p, wp)));
        in_my_way = in_my_way || gap < 9.0f;
    }
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

// Sin ruta: si el compañero anda cerca, lo más probable es que sea él y se vaya.
// Devuelve true mientras valga la pena esperar.
static bool waitForPeer(const Pose &pose, const TelemetrySnapshot &snap) {
    if (!snap.peer.detected || dist(pose.p, peerMarker(snap)) > 24.0f) return false;
    if (stuck_since_ms == 0) stuck_since_ms = millis();
    // Si sé que sigue trabajando, vale la pena esperarlo un poco más
    uint32_t patience = peerIsMoving(coordPeer()) ? 2 * PEER_PATIENCE_MS : PEER_PATIENCE_MS;
    return millis() - stuck_since_ms < patience;
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

// Lo que le cuento al compañero, según lo que estoy haciendo
static void publish(const Pose &pose) {
    Activity act;
    Point to = goal;
    bool carrying = state == ST_TRANSPORTAR || state == ST_ENTREGAR;
    switch (state) {
        case ST_IR: case ST_APUNTAR:            act = ACT_GOING; to = capture.target; break;
        case ST_CAPTURAR: case ST_REAPUNTAR:    act = ACT_CAPTURING; to = capture.target; break;
        case ST_TRANSPORTAR: case ST_ENTREGAR:  act = ACT_CARRYING; to = drop_target; break;
        case ST_SOLTAR: case ST_VERIFICAR:      act = ACT_RELEASING; to = release_from; break;
        case ST_DESPEJAR:                       act = ACT_CLEARING; to = rest.stage; break;
        default:                                act = all_mine_done ? ACT_DONE : ACT_IDLE; to = pose.p; break;
    }
    bool has_cube = act == ACT_GOING || act == ACT_CAPTURING || act == ACT_CARRYING || act == ACT_RELEASING;
    Point waypoint = (state == ST_IR || state == ST_TRANSPORTAR || state == ST_DESPEJAR) ? navWaypoint() : to;
    coordPublish(pose, act, has_cube ? target : COLOR_UNKNOWN, carrying, stuck_since_ms != 0, to, waypoint);
}

void strategyReset() {
    motionStop();
    planNewRound();
    navReset();
    coordReset();
    target = COLOR_UNKNOWN;
    state_name = "ESPERA";
    for (int i = 0; i < NUM_COLORS; i++) { attempts[i] = 0; why[i] = "-"; }
    stats = StrategyStats();
    for (Banned &b : banned) b = Banned{ COLOR_UNKNOWN, Point(), 0, 0 };
    progress_since_ms = 0;
    parking_moves = 0;
    yield_since_ms = 0;
    yield_ignore_until_ms = 0;
    last_select_ms = 0;
    idle_since_ms = 0;
    cleared_until_ms = 0;
    all_mine_done = false;
    have_delivery = false;
    enter(ST_ELEGIR);
}

void strategyStep(const TelemetrySnapshot &seen) {
    // El mundo según este rover: lo que ve la cámara, pero con el compañero donde
    // él mismo dice que está (su pose ya corregida por la latencia).
    static TelemetrySnapshot snap;
    snap = seen;
    const PeerState &peer = coordPeer();
    if (peer.heard) {
        Point marker = advance(peer.pose.p, peer.pose.theta, AXLE_OFFSET);
        snap.peer.col = marker.col;
        snap.peer.row = marker.row;
        snap.peer.theta = peer.pose.theta;
        snap.peer.detected = true;
    }
    Pose pose = motionPredict(snap);
    publish(pose);

    // El que no tiene prioridad evita el tramo que el compañero está recorriendo; el
    // que se está corriendo para no estorbar evita su camino entero, hasta su meta.
    bool i_carry = state == ST_TRANSPORTAR || state == ST_ENTREGAR;
    navSetPeerLeg(peerIsMoving(peer) && !coordIHavePriority(i_carry), peerMarker(snap),
                  state == ST_DESPEJAR ? peer.goal : peer.waypoint);

    // Vigilante de progreso: en los estados en que debería estar moviéndose, si pasa
    // mucho tiempo sin avanzar ni girar, lo que intenta no va a salir.
    bool should_move = state == ST_IR || state == ST_APUNTAR || state == ST_TRANSPORTAR || state == ST_DESPEJAR;
    if (!should_move || progress_since_ms == 0 || dist(pose.p, progress_from) > 2.0f ||
        fabsf(wrapDeg(pose.theta - progress_theta)) > 30.0f) {
        progress_from = pose.p;
        progress_theta = pose.theta;
        progress_since_ms = millis();
    }
    bool no_progress = should_move && millis() - progress_since_ms > NO_PROGRESS_MS;

    if (state == ST_ELEGIR || state == ST_FIN) {
        motionStop();
        if (millis() - last_select_ms < SELECT_PERIOD_MS && last_select_ms != 0) return;
        last_select_ms = millis();
        target = selectTask(snap, pose);
        if (target == COLOR_UNKNOWN) {
            // Nada que pueda hacer ahora. Se sigue mirando: un cubo puede salir de su
            // zona, destaparse el que estaba tapado o liberarse el paso.
            if (idle_since_ms == 0) idle_since_ms = millis();
            bool gave_up = millis() - idle_since_ms > GIVE_UP_MS;
            state_name = all_mine_done || gave_up ? "FIN" : "ESPERAR";
            goal = pose.p;
            state = ST_FIN;
            if (millis() > cleared_until_ms && inTheWay(snap, pose) && findRest(snap, pose, rest)) {
                stats.clears++;
                navReset();
                enter(ST_DESPEJAR);
            }
            return;
        }
        idle_since_ms = 0;
        have_delivery = false;
        navReset();
        enter(ST_IR);
    }

    Point cube = target == COLOR_UNKNOWN ? pose.p : cubePos(snap, target);
    Point depot = target == COLOR_UNKNOWN ? pose.p : snap.depots[target];

    // Dónde queda el cubo respecto al robot (por delante del eje / hacia un costado), de dos formas:
    //  - "visto": rover y cubo del MISMO cuadro de cámara. Exacto aunque los dos se
    //    muevan juntos (cubo en las pinzas), pero con medio segundo de atraso.
    //  - "ahora": con la pose predicha. Vale mientras el cubo esté quieto (antes de tocarlo).
    bool cube_fresh = target != COLOR_UNKNOWN && snap.cubes[target].age_ms < 400;
    Point me_seen = { seen.me.col, seen.me.row };
    Point axle_seen = advance(me_seen, seen.me.theta, -AXLE_OFFSET);
    float bearing_seen = wrapDeg(headingTo(axle_seen, cube) - seen.me.theta) * (float)M_PI / 180.0f;
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
        // Los dos fuimos por el mismo cubo: se lo queda el que ya lo lleva, o el de menor ID
        if (coordPeerClaims(target) && (peer.carrying || ROVER_PEER_ID < ROVER_ID)) { enter(ST_ELEGIR); break; }

        if (no_progress) {
            stats.no_progress++;
            ban(target, capture);
            attempts[target]++;
            enter(ST_ELEGIR);
            break;
        }
        // Estoy sobre el camino del compañero: correrse ya, y después retomar
        if (onPeerPath(snap, pose) && millis() > cleared_until_ms && findRest(snap, pose, rest)) {
            stats.clears++;
            navReset();
            enter(ST_DESPEJAR);
            break;
        }
        if (mustYield(pose, navWaypoint(), snap, false)) {
            state_name = "CEDER";
            goal = pose.p;
            motionStop();
            break;
        }
        NavStatus nav = navGo(pose, capture.stage, capture.dir, snap, COLOR_UNKNOWN);
        goal = navWaypoint();
        if (nav == NAV_NO_PATH) {
            if (waitForPeer(pose, snap)) { state_name = "ESPERAR"; navReset(); break; }
            stats.nav_fail++;
            ban(target, capture);
            attempts[target]++;
            enter(ST_ELEGIR);
            break;
        }
        if (navHasRoute()) stuck_since_ms = 0;
        if (nav == NAV_ARRIVED) enter(ST_APUNTAR);
        break;
    }

    case ST_APUNTAR:
        state_name = "APUNTAR";
        goal = cube;
        if (dist(cube, capture.target) > 1.5f) { enter(ST_ELEGIR); break; }    // El cubo se movió
        // No logra apuntar (no hay lado libre para girar): probar otra captura
        if (millis() - state_since_ms > 8000) {
            stats.aim_timeout++;
            ban(target, capture);
            attempts[target]++;
            enter(ST_ELEGIR);
            break;
        }
        if (aimAt(pose, cube, snap, COLOR_UNKNOWN)) enter(ST_CAPTURAR);
        break;

    case ST_CAPTURAR: {
        state_name = "CAPTURAR";
        goal = cube;
        if (millis() - state_since_ms > 10000) {
            stats.capture_timeout++;
            ban(target, capture);
            release(pose, false);
            break;
        }

        // Ya está contra el frente: a llevarlo
        if (ahead_now < CUBE_AHEAD + 0.6f) {
            motionStop();
            have_delivery = false;
            holding = false;
            hold_used = false;
            navReset();
            enter(ST_TRANSPORTAR);
            break;
        }

        // Antes de que las puntas lleguen al cubo tiene que venir centrado (entra
        // con ±1.25 celdas): si no, una punta lo golpea.
        if (ahead_now < FP_PRONG_TIP + 3.0f && fabsf(aside_now) > 0.9f) {
            motionStop();
            stats.reaims++;
            enter(ST_REAPUNTAR);
            break;
        }

        // El compañero está justo delante: esperar quieto a que pase. (Solo él: los
        // demás cubos ya se comprobaron al elegir esta captura.)
        if (snap.peer.detected &&
            footprintHits(advance(pose.p, pose.theta, 2.0f), pose.theta, peerCenter(snap), PEER_BODY_RADIUS)) {
            motionStop();
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
        state_name = holding ? "HACER_LUGAR" : "TRANSPORTAR";
        if (cube_lost) { stats.cube_lost++; release(pose, false); break; }
        if (no_progress) { stats.no_progress++; release(pose, false); break; }

        // Cómo entrar al destino, ya con el cubo en las pinzas
        if (!have_delivery) {
            motionStop();
            goal = drop_target;
            navPrepare(snap, pose, target);
            have_delivery = bestDrop(snap, target, drop_target, delivery);
            holding = false;
            if (!have_delivery) {
                // El compañero ocupa mi destino: si me toca a mí, le dejo el lugar
                if (moveToHold(snap, pose, target)) { navReset(); break; }
                if (waitForPeer(pose, snap)) { state_name = "ESPERAR"; break; }
                stats.no_drop_route++;
                if (carryToSafety(snap, pose, target, cube_carried)) { navReset(); stuck_since_ms = 0; break; }
                release(pose, false);
            }
            break;
        }

        if (mustYield(pose, navWaypoint(), snap, true)) {
            state_name = "CEDER";
            motionStop();
            break;
        }
        NavStatus nav = navGo(pose, delivery.stage, delivery.dir, snap, target);
        goal = navWaypoint();
        if (nav == NAV_NO_PATH) {
            // No se suelta el cubo a mitad de camino si lo que estorba puede irse
            if (waitForPeer(pose, snap)) { state_name = "ESPERAR"; navReset(); have_delivery = false; break; }
            stats.carry_fail++;
            if (carryToSafety(snap, pose, target, cube_carried)) { navReset(); stuck_since_ms = 0; break; }
            release(pose, false);
            break;
        }
        if (navHasRoute()) stuck_since_ms = 0;
        if (nav == NAV_ARRIVED) enter(ST_ENTREGAR);      // Tramo final recto (al destino o al punto intermedio)
        break;
    }

    case ST_ENTREGAR: {
        state_name = holding ? "HACER_LUGAR" : parking ? "APARTAR" : "ENTREGAR";
        Point drop = holding ? hold_point : drop_target;
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
        if (dist(cube_carried, drop) < 0.3f || passed) {
            if (holding) {
                // En el punto intermedio no se suelta: se vuelve a buscar cómo llegar al destino
                motionStop();
                holding = false;
                have_delivery = false;
                navReset();
                enter(ST_TRANSPORTAR);
            } else {
                release(pose, true);
            }
            break;
        }

        bool blocked = !poseClear(advance(pose.p, pose.theta, 1.5f), pose.theta, snap, target, 0.0f);
        bool timed_out = millis() - state_since_ms > PUSH_TIMEOUT_MS;
        if (cube_lost) stats.cube_lost++;
        else if (timed_out) stats.drop_blocked++;
        if (cube_lost || timed_out) { release(pose, false); break; }
        // Tapado (suele ser el compañero pasando): esperar con el cubo, no soltarlo
        if (blocked) { motionStop(); break; }

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
            stats.deliveries++;
            enter(ST_ELEGIR);                           // Entregado
        } else if (millis() - state_since_ms > 2500) {
            stats.verify_fail++;
            attempts[target]++;                         // No quedó dentro: reintentar o pasar al siguiente
            enter(ST_ELEGIR);
        }
        break;

    case ST_DESPEJAR: {
        // Correrse a un lugar donde no estorbe; después se vuelve a mirar qué hay para hacer
        state_name = "DESPEJAR";
        NavStatus nav = navGo(pose, rest.stage, rest.dir, snap, COLOR_UNKNOWN);
        goal = navWaypoint();
        if (nav != NAV_RUNNING || no_progress || millis() - state_since_ms > 15000) {
            motionStop();
            cleared_until_ms = millis() + 4000;
            enter(ST_FIN);
        }
        break;
    }

    default:
        break;
    }
}

const char* strategyStateName() { return state_name; }
const StrategyStats& strategyStats() { return stats; }
const char* strategyWhy(CubeColor c) { return why[c]; }
CubeColor strategyTarget() { return target; }
Point strategyGoal() { return goal; }
