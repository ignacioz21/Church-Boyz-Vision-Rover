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

static bool drop_aimed = false;      // ENTREGAR: ya apuntó al destino con el cubo
static bool drop_reaimed = false;    // ENTREGAR: ya volvió a apuntar una vez por desvío de costado
static bool retouched[NUM_COLORS];   // Ya se le hizo el retoque a ese cubo (uno por cubo)
// ENTREGAR, al final: se detiene, mira con la cámara dónde quedó el cubo de verdad y,
// si quedó corto, da empujones cortos. (La pose predicha puede adelantarse a la real.)
static uint32_t confirm_since_ms = 0;   // 0 = todavía no está confirmando
static uint32_t nudge_until_ms = 0;
static int nudges = 0;

// Asentar el cubo: antes de girar con él tiene que estar apoyado contra el frente, bien
// adentro de las pinzas. Se mira con la cámara (rover y cubo del mismo cuadro) y, si
// quedó adelantado, se avanza un poco hasta tocarlo.
static bool seat_needed = false;
static uint32_t seat_since_ms = 0, seat_nudge_until_ms = 0, seat_retry_after_ms = 0;
static int seat_nudges = 0;

static void seatStart() {
    seat_needed = true;
    seat_since_ms = millis();
    seat_nudge_until_ms = 0;
    seat_nudges = 0;
}

// true = el cubo está asentado (o no se puede comprobar) y se puede seguir
static bool seatCube(const Pose &pose, const TelemetrySnapshot &snap, CubeColor c, bool cube_fresh, float ahead_seen) {
    uint32_t now = millis();
    if (!seat_needed) {
        // El cubo quedó adelantado (no llegó a tocar el frente, o se corrió hacia las
        // puntas al girar). Tiene que verse así un rato seguido: la imagen llega con
        // atraso y justo después de tomarlo siempre parece más lejos de lo que está.
        static uint32_t slip_since_ms = 0;
        bool slipped = cube_fresh && ahead_seen > CUBE_AHEAD + SEAT_SLIP && now > seat_retry_after_ms;
        if (!slipped) slip_since_ms = 0;
        else if (slip_since_ms == 0) slip_since_ms = now;
        else if (now - slip_since_ms > (uint32_t)motionCal().latency_ms + 150) {
            slip_since_ms = 0;
            seatStart();
            motionStop();
            return false;
        }
        return true;
    }
    if (now < seat_nudge_until_ms) {
        float push = POWER_MIN_MOVE + 0.05f;
        if (poseClear(advance(pose.p, pose.theta, 1.5f), pose.theta, snap, c, 0.0f)) motionDrive(push, push);
        else motionStop();
        seat_since_ms = now;                // La espera cuenta desde que termina el empujón
        return false;
    }
    motionStop();
    if (now - seat_since_ms < (uint32_t)motionCal().latency_ms + 250) return false;
    // Quieto y con la cámara al día
    if (!cube_fresh || ahead_seen <= CUBE_AHEAD + SEAT_TOLERANCE || seat_nudges >= 4) {
        seat_needed = false;
        seat_retry_after_ms = now + 4000;   // Si no se logró, no insistir enseguida
        return true;
    }
    seat_nudges++;
    float speed = motionCal().speed_gain * (POWER_MIN_MOVE + 0.05f);
    // A esta potencia el robot avanza bastante menos de lo que dice la calibración
    seat_nudge_until_ms = now + (uint32_t)constrain((ahead_seen - CUBE_AHEAD) / speed * 1000.0f * 2.5f, 250.0f, 900.0f);
    return false;
}

// Mientras se planifica (puede llevar segundos) hay que seguir avisando: al compañero,
// para que no crea que me apagué, y al monitor.
static void (*keep_alive_hook)() = nullptr;
void strategySetKeepAlive(void (*fn)()) { keep_alive_hook = fn; }
static void keepAlive() {
    coordKeepAlive();
    if (keep_alive_hook) keep_alive_hook();
}

static void enter(State s) {
    state = s;
    drop_aimed = false;
    drop_reaimed = false;
    confirm_since_ms = 0;
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
static bool directDelivery(const Pose &pose, const TelemetrySnapshot &snap, CubeColor c, Point drop, float gap);

static bool bestCapture(const TelemetrySnapshot &snap, const Pose &pose, CubeColor c, Point where, Approach &out) {
    static const int MAX_CANDIDATES = 40, MAX_TRIES = 5;
    static Approach list[MAX_CANDIDATES];
    static float cost[MAX_CANDIDATES];
    int n = 0;

    keepAlive();
    navPrepare(snap, pose, COLOR_UNKNOWN);
    keepAlive();
    Approach app;
    app.target = cubePos(snap, c);
    for (app.dir = 0; app.dir < NAV_DIRS; app.dir++) {
        for (int steps = 4; steps <= 8 && n < MAX_CANDIDATES; steps++) {
            if (!navStage(app.target, app.dir, steps, app.stage)) break;
            // Lo bastante atrás para que las puntas no lleguen al cubo al afinar la puntería
            if (dist(app.stage, app.target) < STAGE_DIST_MIN) continue;
            if (isBanned(c, app)) continue;             // Ya falló hace poco: probar otra
            float k = approachCost(snap, app, CUBE_AHEAD, c);
            if (k < 0.0f) continue;
            // Mejor tomarlo ya mirando hacia donde hay que llevarlo: girar con el cubo
            // en las pinzas es lento y es cuando se puede escapar.
            float turn = fabsf(wrapDeg(headingTo(app.target, where) - headingTo(app.stage, app.target)));
            list[n] = app; cost[n] = k + turn * TURN_WITH_CUBE_COST; n++;
        }
    }

    // El cubo está casi enfrente y el tramo hasta él está libre: tomarlo desde acá, sin
    // ir antes a un punto de preparación. (Caso típico: recién salido, con un cubo
    // delante y sin lugar para maniobrar junto a la línea.)
    float bearing = wrapDeg(headingTo(pose.p, app.target) - pose.theta);
    float reach = dist(pose.p, app.target);
    // Vale de lejos (hay recorrido para afinar la puntería antes de que lleguen las
    // puntas) o de muy cerca si el cubo ya está centrado entre las pinzas: pasa cuando se
    // lo soltó tras un intento fallido y el rover quedó pegado detrás, sin lugar para
    // irse a un punto de preparación.
    float lateral = fabsf(reach * sinf(bearing * (float)M_PI / 180.0f));
    bool far_enough = reach >= FP_PRONG_TIP + 2.5f;
    bool already_in = reach >= CUBE_AHEAD - 0.5f && reach < FP_PRONG_TIP + 2.5f && lateral < 0.9f;
    if (n < MAX_CANDIDATES && fabsf(bearing) <= AIM_MAX_DEG && (far_enough || already_in)) {
        bool clear = true;
        float aim = headingTo(pose.p, app.target);
        for (float d = reach; d >= CUBE_AHEAD && clear; d -= 1.0f) {
            clear = poseClear(advance(app.target, aim, -d), aim, snap, c);
        }
        if (clear) {
            Approach here;
            here.target = app.target;
            here.stage = pose.p;
            here.dir = ((int)lroundf(pose.theta / 45.0f) % NAV_DIRS + NAV_DIRS) % NAV_DIRS;
            float turn = fabsf(wrapDeg(headingTo(here.target, where) - aim));
            if (!isBanned(c, here)) { list[n] = here; cost[n] = reach - CUBE_AHEAD + turn * TURN_WITH_CUBE_COST; n++; }
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
        keepAlive();
        navPrepare(later, captured, c);
        Approach unused;
        // Hay cómo llevarlo si existe una entrada por un punto de preparación, O si
        // desde donde queda tomado se puede girar y empujar derecho. Lo segundo es lo
        // único posible cuando el cubo está cerca de su zona (no hay lugar para el punto
        // de preparación); sin contarlo, ese cubo quedaba sin que nadie lo tomara.
        if (bestDrop(later, c, where, unused) || directDelivery(captured, later, c, where, CUBE_AHEAD)) {
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
// Qué tan caro es atender el cubo 'c' ahora (en celdas): lo que hay que recorrer para
// tomarlo y llevarlo, más un recargo si está en un lugar incómodo (pegado a una línea,
// o con otro cubo cerca de él o de su zona). Sirve para empezar por los fáciles.
static float taskDifficulty(const TelemetrySnapshot &snap, const Pose &pose, CubeColor c) {
    Point cube = cubePos(snap, c);
    float cost = dist(pose.p, cube) + dist(cube, snap.depots[c]);
    float to_line = min(min(cube.col, snap.grid_cols - cube.col), min(cube.row, snap.grid_rows - cube.row));
    if (to_line < 8.0f) cost += (8.0f - to_line) * 4.0f;
    for (int o = 0; o < NUM_COLORS; o++) {
        if (o == c || !pendingCube(snap, (CubeColor)o)) continue;
        Point other = cubePos(snap, (CubeColor)o);
        if (dist(other, cube) < 10.0f || dist(other, snap.depots[c]) < 10.0f) cost += 15.0f;
    }
    return cost;
}

static CubeColor selectTask(const TelemetrySnapshot &snap, const Pose &pose) {
    // Con un plan cargado por la PC se respeta su orden (ya lo optimizó mirando a los
    // dos rovers). Con el reparto de a bordo, mis cubos se atienden del más fácil al
    // más difícil según dónde estoy AHORA, y no en un orden fijo de colores.
    static Plan plan;
    plan = planGet(snap);
    if (planLoadedId() == 0) {
        for (int i = 0; i < plan.n_mine; i++) {
            for (int j = i + 1; j < plan.n_mine; j++) {
                if (taskDifficulty(snap, pose, plan.mine[j]) < taskDifficulty(snap, pose, plan.mine[i])) {
                    CubeColor t = plan.mine[i]; plan.mine[i] = plan.mine[j]; plan.mine[j] = t;
                }
            }
        }
    }
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

// Con el cubo en las pinzas: ¿se puede ir DERECHO al destino desde donde estoy? Vale si
// basta un giro chico para apuntarle y el tramo recto está libre. Entonces no hace
// falta dar la vuelta para entrar por un punto de preparación de la rejilla.
static bool directDelivery(const Pose &pose, const TelemetrySnapshot &snap, CubeColor c, Point drop, float gap) {
    float heading = headingTo(pose.p, drop);
    float turn = wrapDeg(heading - pose.theta);
    if (fabsf(turn) > DIRECT_MAX_TURN) return false;
    float run = dist(pose.p, drop) - gap;           // Lo que avanza el eje hasta dejar el cubo en el destino
    if (run < 0.0f) return false;                   // Ya lo pasó
    if (fabsf(turn) > 4.0f && !pivotClear(pose.p, pose.theta, heading, turn > 0 ? 1 : -1, snap, c, PIVOT_TIGHT)) return false;
    for (float d = 0.0f; d <= run; d += 1.0f) {
        if (!poseClear(advance(pose.p, heading, d), heading, snap, c)) return false;
    }
    return poseClear(advance(pose.p, heading, run), heading, snap, c);
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
    // Sin ruta en curso (planificando, asentando el cubo) no hay tramo que anunciar:
    // el punto del tramo anterior le haría creer al compañero que voy hacia atrás.
    bool routed = state == ST_IR || state == ST_TRANSPORTAR || state == ST_DESPEJAR;
    Point waypoint = !routed ? to : navHasRoute() ? navWaypoint() : pose.p;
    coordPublish(pose, act, has_cube ? target : COLOR_UNKNOWN, carrying, stuck_since_ms != 0, to, waypoint);
}

void strategyReset() {
    motionStop();
    planNewRound();
    navReset();
    coordReset();
    target = COLOR_UNKNOWN;
    state_name = "ESPERA";
    for (int i = 0; i < NUM_COLORS; i++) { attempts[i] = 0; why[i] = "-"; retouched[i] = false; }
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
    geometrySetSelf(pose.p, pose.theta);
    // Si él mismo dice que está quieto, se lo rodea por su forma real
    geometrySetPeerStill(peer.heard && (peer.activity == ACT_IDLE || peer.activity == ACT_DONE));
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
        // Salida ordenada: pegado al compañero (los dos arrancan lado a lado) no salimos
        // a la vez. Sale primero el de menor ID; yo me quedo quieto, y él me rodea
        // sabiendo que no me muevo. Espero hasta que se aleje, o un rato como máximo.
        static uint32_t depart_wait_since_ms = 0;
        bool crowded = snap.peer.detected && peer.heard && peer.activity != ACT_DONE &&
                       ROVER_ID > ROVER_PEER_ID && dist(pose.p, peerCenter(snap)) < DEPART_GAP;
        if (!crowded) depart_wait_since_ms = 0;
        else {
            if (depart_wait_since_ms == 0) depart_wait_since_ms = millis();
            if (millis() - depart_wait_since_ms < DEPART_WAIT_MS) {
                state_name = "ESPERAR";
                goal = pose.p;
                return;
            }
        }
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
        // (y la cámara lo confirma: con la pose predicha sola se da por tomado antes de tiempo)
        if (ahead_now < CUBE_AHEAD + 0.6f && (!cube_fresh || ahead_seen < CUBE_AHEAD + CAPTURE_SEEN_SLACK)) {
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
        // Maniobrando, el cubo ya quedó dentro de su zona: soltarlo ahí. Seguir dando
        // la vuelta para entrarlo "bien" es justo lo que lo vuelve a sacar.
        static int inside_frames = 0;
        static uint32_t inside_seq = 0;
        if (snap.seq != inside_seq) {
            inside_seq = snap.seq;
            inside_frames = (!parking && !holding && cube_fresh && delivered(snap, target)) ? inside_frames + 1 : 0;
        }
        if (inside_frames >= 4) {
            inside_frames = 0;
            if (cubeMargin(cube, target, snap) >= DROP_SAFE_MARGIN) { release(pose, true); break; }
            // Está dentro pero justo en el borde: no se suelta ahí. Se deja de maniobrar
            // y se lo empuja recto al centro.
            motionStop();
            navReset();
            enter(ST_ENTREGAR);
            break;
        }

        if (!seatCube(pose, snap, target, cube_fresh, ahead_seen)) break;

        // Si desde acá se llega derecho, se va derecho: no se sigue una ruta más larga
        // solo porque ya estaba calculada. (Se mira unas veces por segundo.)
        static uint32_t direct_checked_ms = 0;
        if (!holding && millis() - direct_checked_ms > 200) {
            direct_checked_ms = millis();
            if (directDelivery(pose, snap, target, drop_target, gap) && !mustYield(pose, drop_target, snap, true)) {
                motionStop();
                navReset();
                enter(ST_ENTREGAR);
                break;
            }
        }

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
        if (!drop_aimed) {
            if (!seatCube(pose, snap, target, cube_fresh, ahead_seen)) break;
            drop_aimed = aimAt(pose, drop, snap, target);
            // No logró apuntar: empujar igual lleva el cubo a cualquier lado. Se suelta
            // acá y se vuelve a tomar desde un ángulo mejor.
            if (!drop_aimed && millis() - state_since_ms > AIM_WITH_CUBE_MS) {
                stats.aim_timeout++;
                release(pose, false);
            }
            break;
        }

        if (confirm_since_ms != 0) {
            uint32_t now = millis();
            if (now < nudge_until_ms) {
                float push = POWER_MIN_MOVE + 0.05f;
                if (poseClear(advance(pose.p, pose.theta, 1.5f), pose.theta, snap, target, 0.0f)) motionDrive(push, push);
                else motionStop();
                confirm_since_ms = now;         // La espera cuenta desde que termina el empujón
                break;
            }
            motionStop();
            if (now - confirm_since_ms < (uint32_t)motionCal().latency_ms + 250) break;
            // Quieto y con la cámara al día: cuánto le falta al cubo para el centro,
            // medido sobre el rumbo (short_by) y de costado (side)
            float off = wrapDeg(headingTo(cube, drop) - pose.theta) * (float)M_PI / 180.0f;
            float short_by = dist(cube, drop) * cosf(off);
            float side = fabsf(dist(cube, drop) * sinf(off));
            if (!cube_fresh || short_by < DROP_DEPTH_TOL || nudges >= DROP_MAX_NUDGES) { release(pose, true); break; }
            // Desviado de costado y todavía lejos: empujar recto no lo centra. Se vuelve
            // a apuntar al centro (una sola vez) y se sigue desde ahí.
            if (side > DROP_SIDE_TOL && short_by > 2.0f && !drop_reaimed) {
                drop_reaimed = true;
                drop_aimed = false;
                confirm_since_ms = 0;
                state_since_ms = millis();      // El plazo para apuntar empieza de nuevo
                break;
            }
            nudges++;
            // A esta potencia el robot avanza bastante menos de lo que dice la calibración
            float speed = motionCal().speed_gain * (POWER_MIN_MOVE + 0.05f);
            nudge_until_ms = now + (uint32_t)constrain(short_by / speed * 1000.0f * 2.5f, 200.0f, 900.0f);
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
                motionStop();
                confirm_since_ms = millis();
                nudge_until_ms = 0;
                nudges = 0;
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
            // Quedó dentro, pero pegado al límite: un roce lo saca y la regla exige que
            // siga dentro hasta el final. Un retoque (uno solo por cubo): el rover
            // retrocedió recto, así que sigue mirándolo; lo vuelve a tomar y lo empuja
            // al centro.
            if (DROP_RETOUCH && !retouched[target] && cubeMargin(cube, target, snap) < DROP_MIN_MARGIN &&
                fabsf(wrapDeg(headingTo(pose.p, cube) - pose.theta)) < AIM_MAX_DEG) {
                retouched[target] = true;
                stats.retouches++;
                capture.target = cube;
                capture.stage = pose.p;
                drop_target = depot;
                parking = false;
                enter(ST_CAPTURAR);
                break;
            }
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

int strategyRoute(Point *out, int max) {
    int n = 0;
    auto add = [&](Point p) { if (n < max) out[n++] = p; };
    switch (state) {
        case ST_IR:
            n = navRoute(out, max - 3);
            add(capture.stage); add(capture.target); add(drop_target);
            break;
        case ST_APUNTAR: case ST_CAPTURAR: case ST_REAPUNTAR:
            add(capture.target); add(drop_target);
            break;
        case ST_TRANSPORTAR:
            n = navRoute(out, max - 2);
            if (have_delivery) add(delivery.stage);
            add(holding ? hold_point : drop_target);
            break;
        case ST_ENTREGAR:
            add(holding ? hold_point : drop_target);
            break;
        case ST_DESPEJAR:
            n = navRoute(out, max - 1);
            add(rest.stage);
            break;
        default: break;
    }
    return n;
}
