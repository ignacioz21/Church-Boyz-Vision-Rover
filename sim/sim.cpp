// Simulador de la lógica del rover en la PC.
//
// Compila el MISMO código del firmware (geometry, motion, plan, strategy) contra
// una física simple: un rover diferencial, cubos que se empujan y telemetría que
// llega con latencia. Sirve para probar la estrategia sin la cancha.
//
//   ./run.sh [semilla] [plan] [-v]      p. ej.  ./run.sh 3 "P,1,10=rgb,11=" -v
//
// La física es deliberadamente distinta de la calibración de config.h (el robot
// real nunca coincide con sus números): ver TRUE_* abajo.

#include <deque>
#include <random>
#include <string>
#include <Arduino.h>
#include "../rover_10/include/config.h"
#include "../rover_10/include/geometry.h"
#include "../rover_10/include/hardware.h"
#include "../rover_10/include/motion.h"
#include "../rover_10/include/plan.h"
#include "../rover_10/include/strategy.h"
#include "../rover_10/include/telemetry.h"

// --- Física "real" del robot simulado -------------------------------------
static const float TRUE_LATENCY_MS = 480.0f;    // config.h (R10) cree 435
static const float TRUE_SPEED_GAIN = 16.2f;     // config.h (R10) cree 17.8
static const float TRUE_TURN_GAIN = 300.0f;     // config.h (R10) cree 335
static const float MOTOR_TAU_MS = 70.0f;        // Inercia del motor
static const float WHEEL_DEADBAND = 0.15f;      // Por debajo, la rueda no gira
// Forma del robot en su propio marco (origen = centro del marcador, x = frente)
static const float TRUE_AXLE = 1.8f;            // Eje de giro detrás del marcador
static const float BODY_FRONT = 3.1f, BODY_REAR = -3.7f, BODY_HALF_W = 3.25f;
static const float PRONG_TIP = 6.0f, PRONG_SIDE = 2.9f;     // Pinzas: de BODY_FRONT a PRONG_TIP, a +-PRONG_SIDE
static const float SLOT_HALF = 1.25f;           // Holgura lateral del centro del cubo dentro de las pinzas
static const float TRUE_CONTACT = 4.6f;         // Marcador -> centro del cubo apoyado en el frente
static const float CUBE_R = 1.5f;

static const float DEG = 180.0f / M_PI;
static uint32_t now_ms = 1000;
SerialShim Serial;
uint32_t millis() { return now_ms; }
void delay(uint32_t ms) { now_ms += ms; }

// --- Robot y mundo --------------------------------------------------------
// Salida: a 3,75 celdas de la línea (como publica la visión), mirando a la cancha.
// OJO: no se sabe dónde colocará la organización cada rover. El generador oficial los
// dibuja en las filas 17,5 y 25,5, pero como robots de 4 x 3 casillas; con el tamaño
// real (6,5 de ancho), a dificultad alta el cubo rojo —que el generador pone justo
// delante de la salida, entre las filas 19 y 24— quedaría debajo de ellos. Aquí se
// separan lo necesario para que ese cubo quepa en medio con holgura.
static const Point START_10 = { 3.75f, 12.0f }, START_11 = { 3.75f, 31.0f };
static Point rover = START_10;
static float theta = 0.0f;
static float cmd_l = 0, cmd_r = 0, wheel_l = 0, wheel_r = 0;
static Point cubes[NUM_COLORS];
static Point peer = START_11;
static bool peer_present = true;
static int bumps = 0;
static float out_tol = 2.0f;                 // SIM_OUT_TOL: cuántas celdas fuera de la línea se toleran
static int out_ms = 0;                       // Milisegundos con alguna parte del robot fuera de las líneas
static std::string cur_state, bump_info, out_info;     // Para diagnosticar el primer roce

void hardwareInit() {}
void setMotors(float l, float r) { cmd_l = constrain(l, -1.0f, 1.0f); cmd_r = constrain(r, -1.0f, 1.0f); }
void stopMotors() { cmd_l = cmd_r = 0; }
float readUltrasonicCm() { return 999.0f; }
void readFloorSensors(int &a, int &b, int &c, int &d) { a = b = c = d = 0; }
void setLedColor(uint8_t, uint8_t, uint8_t) {}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static bool in_slot[NUM_COLORS];

static void physicsStep(float dt) {
    float k = dt * 1000.0f / MOTOR_TAU_MS;
    wheel_l += (cmd_l - wheel_l) * k;
    wheel_r += (cmd_r - wheel_r) * k;
    float l = fabsf(wheel_l) < WHEEL_DEADBAND ? 0 : wheel_l;
    float r = fabsf(wheel_r) < WHEEL_DEADBAND ? 0 : wheel_r;
    float v = TRUE_SPEED_GAIN * (l + r) / 2.0f;
    float w = TRUE_TURN_GAIN * (r - l) / 2.0f;

    // Gira sobre el eje de las ruedas, no sobre el marcador
    Point axle = advance(rover, theta, -TRUE_AXLE);
    theta += w * dt;
    axle = advance(axle, theta, v * dt);
    rover = advance(axle, theta, TRUE_AXLE);
    float ux = cosf(theta / DEG), uy = -sinf(theta / DEG);

    // Fuera de las líneas: esquinas de la cola, del frente y puntas de las pinzas
    const float corners[6][2] = { { BODY_REAR, BODY_HALF_W }, { BODY_REAR, -BODY_HALF_W }, { BODY_FRONT, BODY_HALF_W },
                                  { BODY_FRONT, -BODY_HALF_W }, { PRONG_TIP, PRONG_SIDE }, { PRONG_TIP, -PRONG_SIDE } };
    bool outside = false;
    for (auto &k : corners) {
        float cx = rover.col + ux * k[0] - uy * k[1], cy = rover.row + uy * k[0] + ux * k[1];
        outside = outside || cx < -out_tol || cx > 43.0f + out_tol || cy < -out_tol || cy > 43.0f + out_tol;
    }
    if (outside && out_ms++ == 0) out_info = cur_state + " t=" + std::to_string((now_ms - 1000) / 1000) + "s";

    for (int c = 0; c < NUM_COLORS; c++) {
        float dx = cubes[c].col - rover.col, dy = cubes[c].row - rover.row;
        float fwd = dx * ux + dy * uy;          // Delante del marcador
        float side = -dx * uy + dy * ux;        // De costado
        bool moved = false;

        bool slot_range = fwd > BODY_FRONT && fwd < PRONG_TIP + CUBE_R;
        if (slot_range && (fabsf(side) < SLOT_HALF || in_slot[c])) {
            // Dentro de las pinzas: el frente lo empuja y las pinzas lo llevan al girar
            in_slot[c] = true;
            if (fwd < TRUE_CONTACT) { fwd = TRUE_CONTACT; moved = true; }
            // Las pinzas lo arrastran de costado, también al pivotar en el sitio
            if (fabsf(side) > SLOT_HALF) { side = clampf(side, -SLOT_HALF, SLOT_HALF); moved = true; }
        } else {
            in_slot[c] = false;
            // Choque con el cuerpo o con una pinza: lo desplaza y cuenta como roce
            float nx = clampf(fwd, BODY_REAR, BODY_FRONT), ny = clampf(side, -BODY_HALF_W, BODY_HALF_W);
            float d_body = hypotf(fwd - nx, side - ny);
            float px = clampf(fwd, BODY_FRONT, PRONG_TIP), py = side > 0 ? PRONG_SIDE : -PRONG_SIDE;
            float d_prong = hypotf(fwd - px, side - py);
            float d = d_body, qx = nx, qy = ny;
            if (d_prong < d_body) { d = d_prong; qx = px; qy = py; }
            // Apoyado por dentro contra la pinza que lo traía: lo mueve, pero no es un roce
            bool inside_gap = slot_range && fabsf(side) < PRONG_SIDE;
            if (d < CUBE_R) {
                if (!inside_gap && bumps++ == 0) bump_info = "cubo " + std::string(1, "rgb"[c]) + " en " + cur_state +
                                              " t=" + std::to_string((now_ms - 1000) / 1000) + "s";
                if (d > 0.01f) { fwd = qx + (fwd - qx) / d * CUBE_R; side = qy + (side - qy) / d * CUBE_R; moved = true; }
            }
        }
        if (moved) {
            cubes[c].col = rover.col + ux * fwd - uy * side;
            cubes[c].row = rover.row + uy * fwd + ux * side;
        }
    }
    if (peer_present && dist(rover, peer) < 7.0f) {
        if (bumps++ == 0) bump_info = "companero en " + cur_state;
    }
}

// --- Telemetría con latencia ----------------------------------------------
struct Frame { uint32_t t; Point rover; float theta; Point cubes[NUM_COLORS]; };
static std::deque<Frame> frames;
static TelemetrySnapshot snap;
static uint32_t seq = 0;

static void cameraCapture() {
    Frame f;
    f.t = now_ms; f.rover = rover; f.theta = fmodf(theta + 3600.0f, 360.0f);
    for (int c = 0; c < NUM_COLORS; c++) f.cubes[c] = cubes[c];
    frames.push_back(f);
}

static void deliverFrames() {
    while (!frames.empty() && frames.front().t + TRUE_LATENCY_MS <= now_ms) {
        const Frame &f = frames.front();
        snap.is_connected = snap.is_valid = true;
        snap.last_packet_time_ms = f.t + (uint32_t)TRUE_LATENCY_MS;
        snap.seq = ++seq;
        snap.phase = PHASE_RUNNING;
        snap.me.id = ROVER_ID; snap.me.col = f.rover.col; snap.me.row = f.rover.row;
        snap.me.theta = f.theta; snap.me.age_ms = 0; snap.me.detected = true;
        snap.peer.id = ROVER_PEER_ID; snap.peer.col = peer.col; snap.peer.row = peer.row;
        snap.peer.detected = peer_present;
        for (int c = 0; c < NUM_COLORS; c++) {
            snap.cubes[c].col = f.cubes[c].col; snap.cubes[c].row = f.cubes[c].row;
            snap.cubes[c].age_ms = 0; snap.cubes[c].detected = true;
        }
        frames.pop_front();
    }
}

bool telemetryGetSnapshot(TelemetrySnapshot &out) { out = snap; return snap.is_valid; }
bool isTelemetryFresh() { return snap.is_valid && now_ms - snap.last_packet_time_ms < TELEMETRY_TIMEOUT_MS; }
void telemetryInit() {}

// --- Generador OFICIAL de posiciones de cubos --------------------------------------
// Port exacto de docs/index.html (https://universidad-cenfotec.github.io/Vision-Rover-Challenge/).
// 'level' es la dificultad de 0 a 1: en 0 los cubos quedan cerca de sus zonas; en 1,
// lejos y con las rutas de transporte cruzándose.
//
// El generador trabaja sobre el tablero de 50 x 50 casillas y lo dibuja girado 90°
// respecto a lo que publica la visión (robots abajo, rojo arriba, verde a la izquierda,
// azul a la derecha). boardToField() pasa a coordenadas de cancha.
namespace official {
struct Rect { float x, y, w, h; };
struct Pt { float x, y; };
static const float INNER_MIN = 5, INNER_MAX = 44, CUBE = 3, MIN_GAP = 2;
static const Rect GOALS[NUM_COLORS] = { { 21, 1, 8, 6 }, { 1, 21, 6, 8 }, { 43, 21, 6, 8 } };   // rojo, verde, azul
static const Rect ROBOTS[2] = { { 19, 46, 4, 3 }, { 27, 46, 4, 3 } };

static bool intersects(Rect a, Rect b) { return a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y; }
static bool tooClose(Rect a, Rect b) {
    return a.x < b.x + b.w + MIN_GAP && a.x + a.w + MIN_GAP > b.x && a.y < b.y + b.h + MIN_GAP && a.y + a.h + MIN_GAP > b.y;
}
static Rect rectOf(Pt c) { return { c.x, c.y, CUBE, CUBE }; }
static Pt centerOf(Rect r) { return { r.x + r.w / 2, r.y + r.h / 2 }; }

static bool valid(const Pt *layout, int i) {
    Rect r = rectOf(layout[i]);
    if (r.x < INNER_MIN || r.y < INNER_MIN || r.x + CUBE - 1 > INNER_MAX || r.y + CUBE - 1 > INNER_MAX) return false;
    for (const Rect &g : GOALS) if (intersects(r, g)) return false;
    for (const Rect &b : ROBOTS) if (intersects(r, b)) return false;
    for (int j = 0; j < NUM_COLORS; j++) if (j != i && tooClose(r, rectOf(layout[j]))) return false;
    return true;
}

static float orient(Pt a, Pt b, Pt c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
static float pointSegDist(Pt p, Pt a, Pt b) {
    float dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    float t = len2 > 0 ? clampf(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0, 1) : 0;
    return hypotf(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}
static float segDistance(Pt a, Pt b, Pt c, Pt d) {
    float o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return 0;
    return min(min(pointSegDist(a, c, d), pointSegDist(b, c, d)), min(pointSegDist(c, a, b), pointSegDist(d, a, b)));
}

// Devuelve las esquinas superiores izquierdas de los cubos (rojo, verde, azul) en casillas del tablero
static void layout(float level, std::mt19937 &rng, Pt *best) {
    auto rnd = [&](float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(rng); };
    const float lo = INNER_MIN, hi = INNER_MAX - CUBE + 1;
    float best_loss = 1e9f;
    float expected_dist = 6.5f + 35 * level;
    int desired_cross = level < .28f ? 0 : level < .53f ? 1 : level < .77f ? 2 : 3;

    for (int attempt = 0; attempt < 700; attempt++) {
        float wave = sinf((float)M_PI * level);
        float jitter = level < .1f ? 2 : (level > .9f ? 2.4f : 3.6f);
        Pt c[NUM_COLORS];
        c[COLOR_RED] = { 23.5f + rnd(-jitter, jitter), 8.5f + 31 * level + rnd(-jitter, jitter) };
        c[COLOR_GREEN] = { 8.5f + 29 * level + rnd(-jitter, jitter), 23 + 5 * wave + rnd(-jitter, jitter) };
        c[COLOR_BLUE] = { 38.5f - 29 * level + rnd(-jitter, jitter), 23 - 5 * wave + rnd(-jitter, jitter) };
        for (Pt &p : c) { p.x = clampf(roundf(p.x), lo, hi); p.y = clampf(roundf(p.y), lo, hi); }
        if (!valid(c, 0) || !valid(c, 1) || !valid(c, 2)) continue;

        float distance = 0;
        Pt center[NUM_COLORS], goal[NUM_COLORS];
        for (int i = 0; i < NUM_COLORS; i++) {
            center[i] = { c[i].x + 1.5f, c[i].y + 1.5f };
            goal[i] = centerOf(GOALS[i]);
            distance += (fabsf(center[i].x - goal[i].x) + fabsf(center[i].y - goal[i].y)) / 3;
        }
        int crossings = 0;
        for (int i = 0; i < NUM_COLORS; i++)
            for (int j = i + 1; j < NUM_COLORS; j++)
                if (segDistance(center[i], goal[i], center[j], goal[j]) < 4) crossings++;

        float loss = fabsf(distance - expected_dist) / 36 * .55f + abs(crossings - desired_cross) / 3.0f * .45f + rnd(0, .06f);
        if (loss < best_loss) { best_loss = loss; for (int i = 0; i < NUM_COLORS; i++) best[i] = c[i]; }
        if (best_loss < .035f && attempt > 40) break;
    }
    if (best_loss < 1e8f) return;

    // Respaldo del generador: posiciones al azar que sean válidas
    for (int i = 0; i < NUM_COLORS; i++) best[i] = { -100, -100 };
    for (int i = 0; i < NUM_COLORS; i++) {
        for (int tries = 0; tries < 10000; tries++) {
            best[i] = { floorf(rnd(lo, hi + 1)), floorf(rnd(lo, hi + 1)) };
            if (valid(best, i)) break;
        }
    }
}

// Casillas del tablero (origen arriba a la izquierda del dibujo) -> celdas de la cancha
// (origen en el marcador 0): giro de 90° y los 3,5 cuadros de margen del tablero.
static Point boardToField(float x, float y) {
    Point p = { 46.5f - y, x - 3.5f };
    return p;
}
}  // namespace official

int main(int argc, char **argv) {
    if (getenv("SIM_OUT_TOL")) out_tol = atof(getenv("SIM_OUT_TOL"));
    int seed = argc > 1 ? atoi(argv[1]) : 1;
    std::string plan = argc > 2 ? argv[2] : "P,1,10=rgb,11=";
    bool verbose = argc > 3;

    snap.depots[COLOR_GREEN] = { 21.5f, 3.75f };
    snap.depots[COLOR_RED] = { 39.25f, 21.5f };
    snap.depots[COLOR_BLUE] = { 21.5f, 39.25f };
    snap.start = { 3.75f, 21.5f };

    std::mt19937 rng(seed);
    const char *level_env = getenv("SIM_LEVEL");
    char layout_name[64];
    if (level_env) {
        // Posiciones del generador oficial, con la dificultad pedida (0 a 1)
        float level = atof(level_env);
        official::Pt corner[NUM_COLORS];
        // El generador no conoce el tamaño real de los rovers: se descartan las
        // disposiciones donde un cubo caería debajo (o pegado) de uno en la salida.
        for (int attempt = 0; attempt < 200; attempt++) {
            official::layout(level, rng, corner);
            bool under_rover = false;
            for (int c = 0; c < NUM_COLORS; c++) {
                cubes[c] = official::boardToField(corner[c].x + 1.5f, corner[c].y + 1.5f);
                for (Point start : { START_10, START_11 }) {
                    Point axle = advance(start, 0.0f, -TRUE_AXLE);
                    under_rover = under_rover || footprintHits(axle, 0.0f, cubes[c], 3.2f);
                }
            }
            if (!under_rover) break;
        }
        snprintf(layout_name, sizeof(layout_name), "oficial D=%.2f", level);
    } else {
        // Cubos al azar, separados entre sí. SIM_MARGIN = qué tan cerca del borde pueden caer (9 por defecto)
        float edge = getenv("SIM_MARGIN") ? atof(getenv("SIM_MARGIN")) : 9.0f;
        std::uniform_real_distribution<float> pos(edge, 43.0f - edge);
        for (int c = 0; c < NUM_COLORS; c++) {
            bool ok = false;
            while (!ok) {
                cubes[c] = { pos(rng), pos(rng) };
                ok = dist(cubes[c], rover) > 8.0f;
                for (int o = 0; o < c; o++) ok = ok && dist(cubes[c], cubes[o]) > 8.0f;
            }
        }
        snprintf(layout_name, sizeof(layout_name), "al azar");
    }
    peer_present = plan.find("11=") + 3 < plan.size();   // Si el 11 tiene tareas, existe (quieto)

    printf("Semilla %d (%s) | plan %s | cubos: R(%.1f,%.1f) G(%.1f,%.1f) B(%.1f,%.1f)\n", seed, layout_name, plan.c_str(),
           cubes[0].col, cubes[0].row, cubes[1].col, cubes[1].row, cubes[2].col, cubes[2].row);

    planLoad(String(plan.c_str()));
    strategyReset();

    const uint32_t T_MAX_MS = 180000;
    uint32_t t0 = now_ms, fin_since = 0;
    std::string last_state;
    while (now_ms - t0 < T_MAX_MS) {
        physicsStep(0.001f);
        now_ms++;
        if (now_ms % 50 == 0) cameraCapture();          // Cámara a 20 Hz
        deliverFrames();
        if (now_ms % 20 == 0 && isTelemetryFresh()) {   // loop() del firmware a 50 Hz
            strategyStep(snap);
            std::string st = strategyStateName();
            if (verbose && st != last_state) {
                printf("  %6.1fs  %-14s rover(%.1f,%.1f) %.0f°\n", (now_ms - t0) / 1000.0f, st.c_str(),
                       rover.col, rover.row, fmodf(theta + 3600.0f, 360.0f));
            }
            // -vv: además, una línea por segundo con pose y órdenes de motor
            if (argc > 3 && std::string(argv[3]) == "-vv" && now_ms % 1000 == 0)
                printf("          t=%5.1f %-13s rover(%.1f,%.1f) %.0f°  motores(%.2f, %.2f)\n", (now_ms - t0) / 1000.0f,
                       st.c_str(), rover.col, rover.row, fmodf(theta + 3600.0f, 360.0f), cmd_l, cmd_r);
            last_state = st;
            cur_state = st;
            // FIN sostenido 2 s = terminó de verdad (FIN también es el estado entre tareas)
            if (st == "FIN") { if (!fin_since) fin_since = now_ms; if (now_ms - fin_since > 2000) break; }
            else fin_since = 0;
        }
    }

    const Plan &p = planGet(snap);
    int delivered = 0;
    for (int i = 0; i < p.n_mine; i++) {
        CubeColor c = p.mine[i];
        float ex = cubeExcess(cubes[c], c, snap);
        delivered += ex == 0.0f;
        printf("  cubo %c: (%.1f, %.1f)  falta %.2f celdas  %s\n", "rgb"[c], cubes[c].col, cubes[c].row, ex,
               ex == 0.0f ? "ENTREGADO" : "NO");
    }
    bool pass = delivered == p.n_mine && bumps == 0 && out_ms == 0;
    printf("%s  %d/%d entregados en %.1f s | roces: %d | estado final: %s\n", pass ? "OK   " : "FALLO",
           delivered, p.n_mine, (now_ms - t0) / 1000.0f, bumps, last_state.c_str());
    if (bumps) printf("       primer roce: %s\n", bump_info.c_str());
    if (out_ms) printf("       fuera de las lineas %d ms; primera vez: %s\n", out_ms, out_info.c_str());
    return pass ? 0 : 1;
}
