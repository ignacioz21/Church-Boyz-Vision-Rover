// El firmware de UN rover empaquetado como biblioteca para el simulador.
//
// Se compila una vez por rover (con las fuentes y el config.h de rover_10/ o de
// rover_11/), así cada uno tiene su propio ID, su calibración y su propio estado
// interno, y no sabe nada del otro salvo lo que le llega por la telemetría: igual
// que en la cancha. El mundo (mundo.cpp) carga las dos bibliotecas.

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "hardware.h"
#include "telemetry.h"
#include "motion.h"
#include "plan.h"
#include "strategy.h"
#include "comms.h"
#include "geometry.h"
#include "nav.h"

#define API extern "C" __attribute__((visibility("default")))

static uint32_t now_ms = 0;
static float cmd_left = 0.0f, cmd_right = 0.0f;
static TelemetrySnapshot current;

// --- Lo que el firmware espera del hardware --------------------------------------
SerialShim Serial;
uint32_t millis() { return now_ms; }
void delay(uint32_t ms) { now_ms += ms; }

void hardwareInit() {}
void setMotors(float l, float r) { cmd_left = constrain(l, -1.0f, 1.0f); cmd_right = constrain(r, -1.0f, 1.0f); }
void stopMotors() { cmd_left = cmd_right = 0.0f; }
float readUltrasonicCm() { return 999.0f; }
void readFloorSensors(int &a, int &b, int &c, int &d) { a = b = c = d = 0; }
void setLedColor(uint8_t, uint8_t, uint8_t) {}
bool imuReady() { return false; }       // El simulador no modela el giroscopio
float gyroZDps() { return 0.0f; }
void gyroRezero() {}
uint16_t gyroFailures() { return 0; }

void telemetryInit() {}
bool telemetryGetSnapshot(TelemetrySnapshot &out) { out = current; return current.is_valid; }
bool isTelemetryFresh() {
    return current.is_valid && current.me.detected &&
           now_ms - current.last_packet_time_ms < TELEMETRY_TIMEOUT_MS;
}

// Canal entre rovers: el mundo se lleva lo que este rover difunde y le trae lo del otro
static String peer_outbox, peer_inbox;
static bool peer_out_pending = false, peer_in_pending = false;
void commsPeerSend(const char *line) { peer_outbox = String(line); peer_out_pending = true; }
bool commsPeerPoll(String &line) {
    if (!peer_in_pending) return false;
    line = peer_inbox;
    peer_in_pending = false;
    return true;
}
bool commsPoll(String &) { return false; }
void commsSend(const char *) {}

// --- Lo que el mundo le pide al rover --------------------------------------------
struct RoverOut {
    float left, right;          // Órdenes de motor
    const char *state;          // Estado de la estrategia
    int target;                 // Cubo en curso (-1 = ninguno)
    float goal_col, goal_row;   // Punto al que se dirige
};

API int rover_id() { return ROVER_ID; }

// Arranque de ronda. 'plan' es el mensaje de la PC ("P,7,10=gb,11=r"), o vacío
// para que el rover use su reparto por defecto.
API void rover_reset(uint32_t now, const char *plan) {
    now_ms = now;
    current = TelemetrySnapshot();
    if (plan && plan[0]) planLoad(String(plan));
    strategyReset();
}

// Una vuelta de loop() con la telemetría que tiene en este momento
API void rover_step(uint32_t now, const TelemetrySnapshot *snap, RoverOut *out) {
    now_ms = now;
    current = *snap;
    if (isTelemetryFresh()) strategyStep(current);
    else motionStop();                  // Como en el .ino: nunca avanzar con una pose vieja
    out->left = cmd_left;
    out->right = cmd_right;
    out->state = strategyStateName();
    out->target = (int)strategyTarget();
    out->goal_col = strategyGoal().col;
    out->goal_row = strategyGoal().row;
}

API void rover_stats(StrategyStats *out) { *out = strategyStats(); }
API const char *rover_why(int color) { return strategyWhy((CubeColor)color); }

// Tareas del plan vigente para este rover, como texto ("gb"); para saber qué le tocó
API void rover_tasks(const TelemetrySnapshot *snap, char *out, int size) {
    const Plan &plan = planGet(*snap);
    int n = 0;
    for (int i = 0; i < plan.n_mine && n < size - 1; i++) out[n++] = "rgb"[plan.mine[i]];
    out[n] = '\0';
}

// Mensaje que este rover difundió desde la última vez (0 = ninguno)
API int rover_peer_out(char *out, int size) {
    if (!peer_out_pending) return 0;
    peer_out_pending = false;
    snprintf(out, size, "%s", peer_outbox.c_str());
    return 1;
}

// Entrega a este rover un mensaje del compañero
API void rover_peer_in(const char *line) {
    peer_inbox = String(line);
    peer_in_pending = true;
}

// --- Para la PC: planificar por adelantado (fase CEREBRO) --------------------------
// cerebro/rutas_pc.py carga esta biblioteca y le pide, para un cubo, la captura y las
// rutas que calcularía ESTE rover: mismo código, misma huella y misma calibración que a
// bordo. Todo va en floats planos para no depender de cómo se empaquetan las estructuras.
//
//   world[30]: 0-1 cancha (cols, rows) · 2-4 zona (largo, fondo) y lado del cubo
//              5-7 este rover: EJE (col, row) y rumbo · 8-11 compañero: visto, marcador (col, row), rumbo
//              12-20 cubos r,g,b: visto, col, row · 21-26 zonas r,g,b: col, row
//   out[80]:   0 cubo (col,row) · 2 desde (col,row) · 4 captura (col,row) · 6 dir · 7 n tramos de ida
//              8.. (col, row, marcha atrás) x 12 · 44 con punto de entrega · 45 entrega (col,row) · 47 dir
//              48 n tramos con cubo · 49.. (col, row) x 12 · 73 dónde queda el rover (col, row, rumbo) · 76 cálculos de ruta
// Devuelve 1 si hay forma de llevar ese cubo, 0 si no.
API float rover_axle_offset() { return AXLE_OFFSET; }

API int rover_preplan(const float *w, int color, float *out) {
    now_ms = 0;
    strategyReset();
    navPlanStatsReset();
    static TelemetrySnapshot s;
    s = TelemetrySnapshot();
    s.is_connected = s.is_valid = true;
    s.grid_cols = w[0]; s.grid_rows = w[1];
    s.depot_length = w[2]; s.depot_depth = w[3]; s.cube_side = w[4];
    Pose pose;
    pose.p.col = w[5]; pose.p.row = w[6]; pose.theta = w[7];
    Point marker = advance(pose.p, pose.theta, AXLE_OFFSET);
    s.me.id = ROVER_ID; s.me.col = marker.col; s.me.row = marker.row; s.me.theta = pose.theta; s.me.detected = true;
    s.peer.id = ROVER_PEER_ID; s.peer.detected = w[8] > 0.5f;
    s.peer.col = w[9]; s.peer.row = w[10]; s.peer.theta = w[11];
    for (int c = 0; c < NUM_COLORS; c++) {
        s.cubes[c].detected = w[12 + 3 * c] > 0.5f;
        s.cubes[c].col = w[13 + 3 * c]; s.cubes[c].row = w[14 + 3 * c];
        s.depots[c].col = w[21 + 2 * c]; s.depots[c].row = w[22 + 2 * c];
    }
    for (int i = 0; i < 80; i++) out[i] = 0.0f;
    static Preplan p;
    if (!strategyPreplan(s, pose, (CubeColor)color, p)) return 0;
    out[0] = p.cube_at.col; out[1] = p.cube_at.row; out[2] = p.from.col; out[3] = p.from.row;
    out[4] = p.stage.col; out[5] = p.stage.row; out[6] = p.dir; out[7] = p.n_go;
    for (int i = 0; i < p.n_go; i++) {
        out[8 + 3 * i] = p.go[i].to.col; out[9 + 3 * i] = p.go[i].to.row; out[10 + 3 * i] = p.go[i].reverse ? 1.0f : 0.0f;
    }
    out[44] = p.has_drop ? 1.0f : 0.0f; out[45] = p.drop_stage.col; out[46] = p.drop_stage.row; out[47] = p.drop_dir;
    out[48] = p.n_carry;
    for (int i = 0; i < p.n_carry; i++) { out[49 + 2 * i] = p.carry[i].to.col; out[50 + 2 * i] = p.carry[i].to.row; }
    out[73] = p.end.p.col; out[74] = p.end.p.row; out[75] = p.end.theta;
    uint32_t n = 0, ms = 0;
    navPlanStats(&n, &ms);
    out[76] = n;
    return 1;
}

// Para probar la carga de planes: id del plan vigente * 100 + rutas guardadas
API int rover_plan_info(const char *msg) {
    if (msg && msg[0]) planLoad(String(msg));
    return planLoadedId() * 100 + planRouteCount();
}

// Con SIM_PLAN_COUNT=1, al terminar la corrida dice cuántas veces este rover calculó una
// ruta: en la placa cada una son ~0,6 s parado, que el simulador no cuenta.
__attribute__((destructor)) static void reportPlanning() {
    if (!getenv("SIM_PLAN_COUNT")) return;
    uint32_t n = 0, ms = 0;
    navPlanStats(&n, &ms);
    fprintf(stderr, "PLAN R%d %u %u\n", ROVER_ID, (unsigned)n, (unsigned)navTravelPlans());
}
