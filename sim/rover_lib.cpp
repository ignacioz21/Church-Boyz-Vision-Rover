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
