// El mundo simulado: uno o dos rovers, los cubos, la cámara y sus latencias.
//
// Cada rover es el firmware real, compilado aparte como biblioteca (rover_lib.cpp con
// las fuentes de rover_10/ y de rover_11/). El mundo solo hace lo que haría la cancha:
// mueve los robots según sus motores, arrastra los cubos, detecta choques y le da a
// cada rover la telemetría con SU latencia. Los rovers no se comunican entre sí.
//
//   ./run.sh [semilla] [plan] [-v | -vv]
//
//   plan = "P,1,10=rgb,11="   solo corre el rover 10 (el 11 no está en la cancha)
//          "P,1,10=g,11=rb"   los dos, con ese reparto
//          "ninguno"          los dos, sin plan de la PC (cada uno usa su reparto por defecto)
//
// La física es deliberadamente distinta (~10 %) de la calibración de cada config.h.

#include <dlfcn.h>
#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <Arduino.h>
#include "../rover_10/include/config.h"
#include "../rover_10/include/geometry.h"
#include "../rover_10/include/strategy.h"

static const float DEG = 180.0f / M_PI;
static uint32_t now_ms = 1000;
SerialShim Serial;
uint32_t millis() { return now_ms; }
void delay(uint32_t ms) { now_ms += ms; }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// --- Forma y física "reales" ------------------------------------------------------
static const float MOTOR_TAU_MS = 70.0f;        // Inercia del motor
static const float WHEEL_DEADBAND = 0.15f;      // Por debajo, la rueda no gira
// Forma del robot en su propio marco (origen = centro del marcador, x = frente)
static const float TRUE_AXLE = 1.8f;            // Eje de giro detrás del marcador
static const float BODY_FRONT = 3.1f, BODY_REAR = -3.7f, BODY_HALF_W = 3.25f;
static const float PRONG_TIP = 6.0f, PRONG_SIDE = 2.9f;     // Pinzas: de BODY_FRONT a PRONG_TIP, a +-PRONG_SIDE
static const float SLOT_HALF = 1.25f;           // Holgura lateral del centro del cubo dentro de las pinzas
static const float TRUE_CONTACT = 4.6f;         // Marcador -> centro del cubo apoyado en el frente
static const float CUBE_R = 1.5f;
static float out_tol = 2.0f;                    // SIM_OUT_TOL: cuántas celdas fuera de la línea se toleran

// Salida: a 3,75 celdas de la línea (como publica la visión), mirando a la cancha.
// OJO: no se sabe dónde colocará la organización cada rover. El generador oficial los
// dibuja en las filas 17,5 y 25,5, pero como robots de 4 x 3 casillas; con el tamaño
// real (6,5 de ancho), a dificultad alta el cubo rojo —que el generador pone justo
// delante de la salida, entre las filas 19 y 24— quedaría debajo de ellos. Aquí se
// separan lo necesario para que ese cubo quepa en medio con holgura.
static const Point START_10 = { 3.75f, 12.0f }, START_11 = { 3.75f, 31.0f };

// --- Un rover: su firmware (biblioteca) y su cuerpo en el mundo ---------------------
struct RoverOut { float left, right; const char *state; int target; float goal_col, goal_row; };

struct Robot {
    int id = 0;
    // Firmware
    void (*reset)(uint32_t, const char *) = nullptr;
    void (*step)(uint32_t, const TelemetrySnapshot *, RoverOut *) = nullptr;
    void (*stats)(StrategyStats *) = nullptr;
    const char *(*why)(int) = nullptr;
    void (*tasks)(const TelemetrySnapshot *, char *, int) = nullptr;
    int (*peer_out)(char *, int) = nullptr;
    void (*peer_in)(const char *) = nullptr;
    // Física real de ESTE robot (su config.h cree otros números)
    float true_latency = 480, true_speed = 16.2f, true_turn = 300;
    // Cuerpo
    Point marker;
    float theta = 0;
    float cmd_l = 0, cmd_r = 0, wheel_l = 0, wheel_r = 0;
    bool in_slot[NUM_COLORS] = { false, false, false };
    // Telemetría que tiene ahora
    TelemetrySnapshot snap;
    size_t next_frame = 0;
    // Registro
    std::string state, bump_info, out_info;
    std::map<std::string, int> state_ms;
    float travelled = 0, turned = 0;
    int bumps = 0, out_ms = 0, crash_ms = 0;
    uint32_t fin_since = 0;
};

static std::vector<Robot> robots;
static Point cubes[NUM_COLORS];
static std::string crash_info;                  // Primer choque entre rovers

static bool loadRover(Robot &r, const char *path) {
    void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!lib) { fprintf(stderr, "No se pudo cargar %s: %s\n", path, dlerror()); return false; }
    r.id = ((int (*)())dlsym(lib, "rover_id"))();
    r.reset = (void (*)(uint32_t, const char *))dlsym(lib, "rover_reset");
    r.step = (void (*)(uint32_t, const TelemetrySnapshot *, RoverOut *))dlsym(lib, "rover_step");
    r.stats = (void (*)(StrategyStats *))dlsym(lib, "rover_stats");
    r.why = (const char *(*)(int))dlsym(lib, "rover_why");
    r.tasks = (void (*)(const TelemetrySnapshot *, char *, int))dlsym(lib, "rover_tasks");
    r.peer_out = (int (*)(char *, int))dlsym(lib, "rover_peer_out");
    r.peer_in = (void (*)(const char *))dlsym(lib, "rover_peer_in");
    return true;
}

// Punto del contorno del robot (marco del marcador) -> cancha
static Point bodyToField(const Robot &r, float x, float y) {
    float ux = cosf(r.theta / DEG), uy = -sinf(r.theta / DEG);
    Point p = { r.marker.col + ux * x - uy * y, r.marker.row + uy * x + ux * y };
    return p;
}

// ¿El punto 'p' cae sobre el cuerpo o las pinzas de 'r'?
static bool pointOnRobot(const Robot &r, Point p) {
    float ux = cosf(r.theta / DEG), uy = -sinf(r.theta / DEG);
    float dx = p.col - r.marker.col, dy = p.row - r.marker.row;
    float x = dx * ux + dy * uy, y = -dx * uy + dy * ux;
    if (x >= BODY_REAR && x <= BODY_FRONT && fabsf(y) <= BODY_HALF_W) return true;
    return x >= BODY_FRONT && x <= PRONG_TIP && fabsf(fabsf(y) - PRONG_SIDE) < 0.3f;
}

static bool robotsOverlap(const Robot &a, const Robot &b) {
    // Contorno muestreado: esquinas, puntas y puntos medios
    static const float OUTLINE[][2] = {
        { BODY_REAR, BODY_HALF_W }, { BODY_REAR, 0 }, { BODY_REAR, -BODY_HALF_W },
        { 0, BODY_HALF_W }, { 0, -BODY_HALF_W }, { BODY_FRONT, BODY_HALF_W }, { BODY_FRONT, -BODY_HALF_W },
        { BODY_FRONT, 0 }, { PRONG_TIP, PRONG_SIDE }, { PRONG_TIP, -PRONG_SIDE },
        { (BODY_FRONT + PRONG_TIP) / 2, PRONG_SIDE }, { (BODY_FRONT + PRONG_TIP) / 2, -PRONG_SIDE },
    };
    for (const auto &k : OUTLINE) {
        if (pointOnRobot(b, bodyToField(a, k[0], k[1])) || pointOnRobot(a, bodyToField(b, k[0], k[1]))) return true;
    }
    return false;
}

static void physicsStep(Robot &r, float dt) {
    float k = dt * 1000.0f / MOTOR_TAU_MS;
    r.wheel_l += (r.cmd_l - r.wheel_l) * k;
    r.wheel_r += (r.cmd_r - r.wheel_r) * k;
    float l = fabsf(r.wheel_l) < WHEEL_DEADBAND ? 0 : r.wheel_l;
    float w_r = fabsf(r.wheel_r) < WHEEL_DEADBAND ? 0 : r.wheel_r;
    float v = r.true_speed * (l + w_r) / 2.0f;
    float w = r.true_turn * (w_r - l) / 2.0f;

    // Gira sobre el eje de las ruedas, no sobre el marcador
    Point old_marker = r.marker;
    float old_theta = r.theta;
    Point axle = advance(r.marker, r.theta, -TRUE_AXLE);
    r.theta += w * dt;
    axle = advance(axle, r.theta, v * dt);
    r.marker = advance(axle, r.theta, TRUE_AXLE);

    // Choque con el otro rover: no lo atraviesa, y se cuenta
    for (Robot &o : robots) {
        if (&o == &r || !robotsOverlap(r, o)) continue;
        r.marker = old_marker;
        r.theta = old_theta;
        v = w = 0;
        if (r.crash_ms++ == 0 && crash_info.empty())
            crash_info = "R" + std::to_string(r.id) + " en " + r.state + " contra R" + std::to_string(o.id) + " en " + o.state +
                         " t=" + std::to_string((now_ms - 1000) / 1000) + "s";
    }
    r.travelled += fabsf(v) * dt;
    r.turned += fabsf(w) * dt;
    float ux = cosf(r.theta / DEG), uy = -sinf(r.theta / DEG);

    // Fuera de las líneas: esquinas de la cola, del frente y puntas de las pinzas
    static const float CORNERS[6][2] = { { BODY_REAR, BODY_HALF_W }, { BODY_REAR, -BODY_HALF_W }, { BODY_FRONT, BODY_HALF_W },
                                         { BODY_FRONT, -BODY_HALF_W }, { PRONG_TIP, PRONG_SIDE }, { PRONG_TIP, -PRONG_SIDE } };
    bool outside = false;
    for (const auto &c : CORNERS) {
        Point p = bodyToField(r, c[0], c[1]);
        outside = outside || p.col < -out_tol || p.col > 43.0f + out_tol || p.row < -out_tol || p.row > 43.0f + out_tol;
    }
    if (outside && r.out_ms++ == 0) r.out_info = r.state + " t=" + std::to_string((now_ms - 1000) / 1000) + "s";

    for (int c = 0; c < NUM_COLORS; c++) {
        float dx = cubes[c].col - r.marker.col, dy = cubes[c].row - r.marker.row;
        float fwd = dx * ux + dy * uy;          // Delante del marcador
        float side = -dx * uy + dy * ux;        // De costado
        bool moved = false;

        bool slot_range = fwd > BODY_FRONT && fwd < PRONG_TIP + CUBE_R;
        if (slot_range && (fabsf(side) < SLOT_HALF || r.in_slot[c])) {
            // Dentro de las pinzas: el frente lo empuja y las pinzas lo arrastran de
            // costado, también al pivotar en el sitio
            r.in_slot[c] = true;
            if (fwd < TRUE_CONTACT) { fwd = TRUE_CONTACT; moved = true; }
            if (fabsf(side) > SLOT_HALF) { side = clampf(side, -SLOT_HALF, SLOT_HALF); moved = true; }
        } else {
            r.in_slot[c] = false;
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
                if (!inside_gap && r.bumps++ == 0)
                    r.bump_info = "cubo " + std::string(1, "rgb"[c]) + " en " + r.state + " t=" + std::to_string((now_ms - 1000) / 1000) + "s";
                if (d > 0.01f) { fwd = qx + (fwd - qx) / d * CUBE_R; side = qy + (side - qy) / d * CUBE_R; moved = true; }
            }
        }
        if (moved) {
            cubes[c].col = r.marker.col + ux * fwd - uy * side;
            cubes[c].row = r.marker.row + uy * fwd + ux * side;
        }
    }
}

// --- Radio entre rovers: los mensajes llegan con un atraso y a veces se pierden -----
//   SIM_PEER=0         los rovers no se escuchan (solo cámara y plan)
//   SIM_PEER_LOSS=0.1  fracción de mensajes que se pierde
struct Radio { uint32_t deliver_ms; size_t to; std::string line; };
static std::deque<Radio> radio;
static bool radio_on = true;
static float radio_loss = 0.10f;
static const uint32_t RADIO_DELAY_MS = 40;
static std::mt19937 radio_rng(12345);
static int radio_sent = 0, radio_lost = 0;

static void radioStep() {
    for (size_t i = 0; i < robots.size(); i++) {
        char line[160];
        if (!robots[i].peer_out(line, sizeof(line)) || !radio_on || robots.size() < 2) continue;
        radio_sent++;
        if (std::uniform_real_distribution<float>(0, 1)(radio_rng) < radio_loss) { radio_lost++; continue; }
        radio.push_back({ now_ms + RADIO_DELAY_MS, 1 - i, line });
    }
    while (!radio.empty() && radio.front().deliver_ms <= now_ms) {
        robots[radio.front().to].peer_in(radio.front().line.c_str());
        radio.pop_front();
    }
}

// --- Cámara: un cuadro cada 50 ms; cada rover lo recibe con su latencia -------------
struct Frame {
    uint32_t t;
    Point marker[2];
    float theta[2];
    Point cubes[NUM_COLORS];
};
static std::deque<Frame> frames;
static size_t frames_dropped = 0;       // Cuadros ya descartados del frente de la cola
static uint32_t seq = 0;
static TelemetrySnapshot field;         // Lo que no cambia: cancha y zonas

static void cameraCapture() {
    Frame f;
    f.t = now_ms;
    for (size_t i = 0; i < robots.size(); i++) {
        f.marker[i] = robots[i].marker;
        f.theta[i] = fmodf(robots[i].theta + 3600.0f, 360.0f);
    }
    for (int c = 0; c < NUM_COLORS; c++) f.cubes[c] = cubes[c];
    frames.push_back(f);
}

static void deliverFrames() {
    for (size_t i = 0; i < robots.size(); i++) {
        Robot &r = robots[i];
        while (r.next_frame - frames_dropped < frames.size() &&
               frames[r.next_frame - frames_dropped].t + r.true_latency <= now_ms) {
            const Frame &f = frames[r.next_frame - frames_dropped];
            TelemetrySnapshot &s = r.snap;
            s = field;
            s.is_connected = s.is_valid = true;
            s.last_packet_time_ms = f.t + (uint32_t)r.true_latency;
            s.seq = (uint32_t)r.next_frame + 1;
            s.phase = PHASE_RUNNING;
            s.me.id = r.id; s.me.col = f.marker[i].col; s.me.row = f.marker[i].row;
            s.me.theta = f.theta[i]; s.me.age_ms = 0; s.me.detected = true;
            s.peer.detected = robots.size() > 1;
            if (s.peer.detected) {
                size_t o = 1 - i;
                s.peer.id = robots[o].id; s.peer.col = f.marker[o].col; s.peer.row = f.marker[o].row;
                s.peer.theta = f.theta[o]; s.peer.age_ms = 0;
            }
            for (int c = 0; c < NUM_COLORS; c++) {
                s.cubes[c].col = f.cubes[c].col; s.cubes[c].row = f.cubes[c].row;
                s.cubes[c].age_ms = 0; s.cubes[c].detected = true;
            }
            r.next_frame++;
        }
    }
    // Se descartan los cuadros que ya recibieron todos
    size_t min_next = robots[0].next_frame;
    for (const Robot &r : robots) min_next = min(min_next, r.next_frame);
    while (frames_dropped < min_next) { frames.pop_front(); frames_dropped++; }
}

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
    if (getenv("SIM_PEER")) radio_on = atoi(getenv("SIM_PEER")) != 0;
    if (getenv("SIM_PEER_LOSS")) radio_loss = atof(getenv("SIM_PEER_LOSS"));
    int seed = argc > 1 ? atoi(argv[1]) : 1;
    std::string plan = argc > 2 ? argv[2] : "P,1,10=rgb,11=";
    std::string flag = argc > 3 ? argv[3] : "";
    bool verbose = flag == "-v" || flag == "-vv", very = flag == "-vv";

    field.depots[COLOR_GREEN] = { 21.5f, 3.75f };
    field.depots[COLOR_RED] = { 39.25f, 21.5f };
    field.depots[COLOR_BLUE] = { 21.5f, 39.25f };
    field.start = { 3.75f, 21.5f };

    // Rovers en la cancha: el 10 siempre; el 11 salvo que el plan no le dé nada
    bool no_plan = plan == "ninguno";
    bool with_11 = no_plan || plan.find("11=") + 3 < plan.size();
    const char *dir = getenv("SIM_LIB_DIR") ? getenv("SIM_LIB_DIR") : "/tmp";
    robots.resize(with_11 ? 2 : 1);
    if (!loadRover(robots[0], (std::string(dir) + "/librover_10.so").c_str())) return 2;
    robots[0].marker = START_10;
    robots[0].true_latency = 480; robots[0].true_speed = 16.2f; robots[0].true_turn = 300;      // config.h cree 435 / 17.8 / 335
    if (with_11) {
        if (!loadRover(robots[1], (std::string(dir) + "/librover_11.so").c_str())) return 2;
        robots[1].marker = START_11;
        robots[1].true_latency = 540; robots[1].true_speed = 14.2f; robots[1].true_turn = 250;  // config.h cree 490 / 15.6 / 278
    }

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
                ok = dist(cubes[c], START_10) > 8.0f && dist(cubes[c], START_11) > 8.0f;
                for (int o = 0; o < c; o++) ok = ok && dist(cubes[c], cubes[o]) > 8.0f;
            }
        }
        snprintf(layout_name, sizeof(layout_name), "al azar");
    }

    if (flag == "--disposicion") {
        // Solo la disposición inicial, en el formato del mensaje de la visión: es lo
        // que el planificador de la PC vería en IDLE.
        printf("{\"grid\":{\"cols\":43,\"rows\":43},\"cube_side\":3.0,\"depot_size\":{\"length\":10.0,\"depth\":7.5},\"rovers\":[");
        for (size_t i = 0; i < 2; i++) {
            Point s = i == 0 ? START_10 : START_11;
            printf("%s{\"id\":%d,\"col\":%.2f,\"row\":%.2f,\"theta\":0.0}", i ? "," : "", i == 0 ? 10 : 11, s.col, s.row);
        }
        printf("],\"cubes\":[");
        const char *names[] = { "red", "green", "blue" };
        for (int c = 0; c < NUM_COLORS; c++) printf("%s{\"color\":\"%s\",\"col\":%.2f,\"row\":%.2f}", c ? "," : "", names[c], cubes[c].col, cubes[c].row);
        printf("],\"depots\":[");
        for (int c = 0; c < NUM_COLORS; c++) printf("%s{\"color\":\"%s\",\"col\":%.2f,\"row\":%.2f}", c ? "," : "", names[c], field.depots[c].col, field.depots[c].row);
        printf("]}\n");
        return 0;
    }

    printf("Semilla %d (%s) | plan %s | %zu rover(s) | cubos: R(%.1f,%.1f) G(%.1f,%.1f) B(%.1f,%.1f)\n", seed, layout_name,
           plan.c_str(), robots.size(), cubes[0].col, cubes[0].row, cubes[1].col, cubes[1].row, cubes[2].col, cubes[2].row);

    for (Robot &r : robots) r.reset(now_ms, no_plan ? "" : plan.c_str());

    // Momento en que cada cubo quedó en su zona (y no volvió a salir)
    Point cubes0[NUM_COLORS];
    float in_since[NUM_COLORS];
    for (int c = 0; c < NUM_COLORS; c++) {
        cubes0[c] = cubes[c];
        in_since[c] = cubeExcess(cubes[c], (CubeColor)c, field) == 0.0f ? 0.0f : -1.0f;
    }

    const uint32_t T_MAX_MS = 180000;
    uint32_t t0 = now_ms;
    while (now_ms - t0 < T_MAX_MS) {
        for (Robot &r : robots) physicsStep(r, 0.001f);
        now_ms++;
        if (now_ms % 50 == 0) {
            cameraCapture();                            // Cámara a 20 Hz
            for (int c = 0; c < NUM_COLORS; c++) {
                bool in = cubeExcess(cubes[c], (CubeColor)c, field) == 0.0f;
                if (!in) in_since[c] = -1.0f;
                else if (in_since[c] < 0) in_since[c] = (now_ms - t0) / 1000.0f;
            }
        }
        deliverFrames();
        if (now_ms % 20 != 0) continue;                 // loop() de cada firmware a 50 Hz

        bool all_done = true;
        for (Robot &r : robots) {
            RoverOut out = { 0, 0, "SIN_TELEMETRIA", -1, 0, 0 };
            r.step(now_ms, &r.snap, &out);
            r.cmd_l = out.left; r.cmd_r = out.right;
            std::string st = out.state;
            r.state_ms[st] += 20;
            if (verbose && st != r.state)
                printf("  %6.1fs  R%d %-14s (%.1f,%.1f) %.0f°\n", (now_ms - t0) / 1000.0f, r.id, st.c_str(),
                       r.marker.col, r.marker.row, fmodf(r.theta + 3600.0f, 360.0f));
            if (very && now_ms % 1000 == 0)
                printf("          t=%5.1f R%d %-13s (%.1f,%.1f) %.0f°  motores(%.2f, %.2f)\n", (now_ms - t0) / 1000.0f, r.id,
                       st.c_str(), r.marker.col, r.marker.row, fmodf(r.theta + 3600.0f, 360.0f), r.cmd_l, r.cmd_r);
            r.state = st;
            // FIN sostenido 2 s = terminó de verdad (FIN también es el estado entre tareas)
            if (st == "FIN") { if (!r.fin_since) r.fin_since = now_ms; }
            else r.fin_since = 0;
            all_done = all_done && r.fin_since && now_ms - r.fin_since > 2000;
        }
        radioStep();
        if (all_done) break;
    }

    // --- Resultado ---
    int delivered = 0, bumps = 0, out_ms = 0, crash_ms = 0;
    for (int c = 0; c < NUM_COLORS; c++) {
        float ex = cubeExcess(cubes[c], (CubeColor)c, field);
        delivered += ex == 0.0f;
        printf("  cubo %c: (%.1f, %.1f)  falta %.2f celdas  %s\n", "rgb"[c], cubes[c].col, cubes[c].row, ex, ex == 0.0f ? "ENTREGADO" : "NO");
    }
    for (const Robot &r : robots) { bumps += r.bumps; out_ms += r.out_ms; crash_ms += r.crash_ms; }
    bool pass = delivered == NUM_COLORS && bumps == 0 && out_ms == 0 && crash_ms == 0;
    float t_end = (now_ms - t0) / 1000.0f;
    printf("%s  %d/3 entregados en %.1f s | roces: %d | choques entre rovers: %d ms | fuera: %d ms |", pass ? "OK   " : "FALLO",
           delivered, t_end, bumps, crash_ms, out_ms);
    for (const Robot &r : robots) printf(" R%d %s", r.id, r.state.c_str());
    printf("\n");
    for (const Robot &r : robots) {
        if (r.bumps) printf("       R%d primer roce: %s\n", r.id, r.bump_info.c_str());
        if (r.out_ms) printf("       R%d fuera de las lineas %d ms; primera vez: %s\n", r.id, r.out_ms, r.out_info.c_str());
    }
    if (crash_ms) printf("       primer choque: %s\n", crash_info.c_str());

    if (getenv("SIM_JSON")) {
        // Una línea con todo, para analizar.py
        printf("JSON {\"seed\":%d,\"layout\":\"%s\",\"plan\":\"%s\",\"n_rovers\":%zu,\"radio\":%s,\"ok\":%s,\"n_delivered\":%d,\"t_end\":%.1f,"
               "\"bumps\":%d,\"out_ms\":%d,\"crash_ms\":%d,\"crash\":\"%s\",",
               seed, layout_name, plan.c_str(), robots.size(), radio_on ? "true" : "false", pass ? "true" : "false", delivered, t_end, bumps, out_ms, crash_ms, crash_info.c_str());
        printf("\"cubes0\":[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]],\"cubes1\":[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]],\"t_in\":[%.1f,%.1f,%.1f],\"excess\":[%.2f,%.2f,%.2f],\"rovers\":[",
               cubes0[0].col, cubes0[0].row, cubes0[1].col, cubes0[1].row, cubes0[2].col, cubes0[2].row,
               cubes[0].col, cubes[0].row, cubes[1].col, cubes[1].row, cubes[2].col, cubes[2].row,
               in_since[0], in_since[1], in_since[2],
               cubeExcess(cubes[0], COLOR_RED, field), cubeExcess(cubes[1], COLOR_GREEN, field), cubeExcess(cubes[2], COLOR_BLUE, field));
        for (size_t i = 0; i < robots.size(); i++) {
            Robot &r = robots[i];
            StrategyStats k;
            r.stats(&k);
            char tasks[8];
            r.tasks(&r.snap, tasks, sizeof(tasks));
            printf("%s{\"id\":%d,\"tasks\":\"%s\",\"final\":\"%s\",\"bumps\":%d,\"bump\":\"%s\",\"out_ms\":%d,\"out\":\"%s\",\"crash_ms\":%d,"
                   "\"travelled\":%.1f,\"turned\":%.0f,\"why\":[\"%s\",\"%s\",\"%s\"],",
                   i ? "," : "", r.id, tasks, r.state.c_str(), r.bumps, r.bump_info.c_str(), r.out_ms, r.out_info.c_str(), r.crash_ms,
                   r.travelled, r.turned, r.why(0), r.why(1), r.why(2));
            printf("\"stats\":{\"deliveries\":%d,\"parks\":%d,\"helps\":%d,\"clears\":%d,\"reaims\":%d,\"no_progress\":%d,\"nav_fail\":%d,\"aim_timeout\":%d,\"capture_timeout\":%d,"
                   "\"no_drop_route\":%d,\"carry_fail\":%d,\"cube_lost\":%d,\"drop_blocked\":%d,\"verify_fail\":%d},\"state_ms\":{",
                   k.deliveries, k.parks, k.helps, k.clears, k.reaims, k.no_progress, k.nav_fail, k.aim_timeout, k.capture_timeout, k.no_drop_route,
                   k.carry_fail, k.cube_lost, k.drop_blocked, k.verify_fail);
            bool first = true;
            for (auto &kv : r.state_ms) { printf("%s\"%s\":%d", first ? "" : ",", kv.first.c_str(), kv.second); first = false; }
            printf("}}");
        }
        printf("]}\n");
    }
    return pass ? 0 : 1;
}
