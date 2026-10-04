/**
 * Rover 11 — firmware (Vision Rover Challenge)
 *
 *   Core 0: telemetry.cpp lee la visión (TCP 2026) y guarda el último estado.
 *   Core 1: este loop() — comandos, seguridad, estrategia y reporte de estado.
 *
 * La lógica de competencia está en src/strategy.cpp y corre a bordo.
 */

#include "include/config.h"
#include "include/types.h"
#include "include/hardware.h"
#include "include/telemetry.h"
#include "include/comms.h"
#include "include/geometry.h"
#include "include/motion.h"
#include "include/plan.h"
#include "include/strategy.h"

#define LOOP_PERIOD_MS      20      // ~50 Hz
#define STATUS_PERIOD_MS    200     // Estado al cerebro
#define TEST_CMD_TIMEOUT_MS 500     // Un "M" de prueba caduca si no se renueva
#define GOTO_TIMEOUT_MS     20000

// Modos de PRÁCTICA (solo desarrollo; en un intento nadie manda comandos)
static uint32_t test_until_ms = 0;      // Motores directos (f, M)
static bool practice_run = false;       // Estrategia forzada sin esperar la fase (S)
static bool goto_active = false;        // Ir a un punto (G)
static Point goto_target;
static uint32_t goto_since_ms = 0;

static bool strategy_running = false;

static const char* phaseName(CompetitionPhase p) {
    switch (p) {
        case PHASE_READY:    return "READY";
        case PHASE_RUNNING:  return "RUNNING";
        case PHASE_FINISHED: return "FINISHED";
        default:             return "IDLE";
    }
}

static void stopAll() {
    test_until_ms = 0;
    practice_run = false;
    goto_active = false;
    motionStop();
}

static void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    char c = cmd.charAt(0);
    int a = cmd.indexOf(',');
    int b = cmd.indexOf(',', a + 1);

    if (c == 'r') {                             // Parar todo
        stopAll();
        Serial.println("[CMD] STOP");
    } else if (c == 'P') {                      // Plan de la PC: solo antes de arrancar
        if (strategy_running) {
            Serial.println("[PLAN] Ignorado: la ronda ya esta en curso");
        } else if (planLoad(cmd)) {
            Serial.printf("[PLAN] Cargado: %s\n", cmd.c_str());
        } else {
            Serial.printf("[PLAN] Mal formado: %s\n", cmd.c_str());
        }
    } else if (c == 'S') {                      // Práctica: correr la estrategia ya
        stopAll();
        practice_run = true;
        Serial.println("[CMD] Practica: estrategia iniciada");
    } else if (c == 'G' && a > 0 && b > a) {    // Práctica: "G,col,row"
        stopAll();
        goto_target.col = cmd.substring(a + 1, b).toFloat();
        goto_target.row = cmd.substring(b + 1).toFloat();
        goto_active = true;
        goto_since_ms = millis();
        Serial.printf("[CMD] Practica: ir a (%.1f, %.1f)\n", goto_target.col, goto_target.row);
    } else if (c == 'L') {                      // Práctica: calibrar (bloquea ~4 s)
        stopAll();
        motionCalibrate();
    } else if (c == 'f') {                      // Prueba: avanzar 1 s
        stopAll();
        setMotors(0.4f, 0.4f);
        test_until_ms = millis() + 1000;
    } else if (c == 'A') {                      // Prueba: arco suave 2 s ("A" izquierda, "A,r" derecha)
        stopAll();
        bool right = cmd.indexOf('r') > 0;
        setMotors(right ? 0.45f : 0.30f, right ? 0.30f : 0.45f);
        test_until_ms = millis() + 2000;
    } else if (c == 'M' && a > 0 && b > a) {    // Prueba: "M,izq,der"
        setMotors(cmd.substring(a + 1, b).toFloat(), cmd.substring(b + 1).toFloat());
        test_until_ms = millis() + TEST_CMD_TIMEOUT_MS;
    } else {
        Serial.printf("[CMD] Desconocido: %s  (r | P | S | G,col,row | L | f | A | A,r | M,izq,der)\n", cmd.c_str());
    }
}

static void sendStatus(const TelemetrySnapshot &snap, bool fresh, const char* state) {
    static const char COLOR_CHAR[] = { 'r', 'g', 'b' };
    CubeColor target = strategy_running ? strategyTarget() : COLOR_UNKNOWN;
    Point goal = goto_active ? goto_target : strategyGoal();
    const MotionCal &cal = motionCal();
    char line[360];
    snprintf(line, sizeof(line),
             "{\"id\":%d,\"state\":\"%s\",\"fresh\":%s,\"link\":%s,\"phase\":\"%s\","
             "\"col\":%.2f,\"row\":%.2f,\"theta\":%.1f,\"seq\":%u,"
             "\"plan\":%d,\"task\":\"%c\",\"gc\":%.1f,\"gr\":%.1f,"
             "\"lat\":%.0f,\"vg\":%.1f,\"wg\":%.0f}",
             ROVER_ID, state, fresh ? "true" : "false", snap.is_connected ? "true" : "false",
             phaseName(snap.phase), snap.me.col, snap.me.row, snap.me.theta, (unsigned)snap.seq,
             planLoadedId(), target == COLOR_UNKNOWN ? '-' : COLOR_CHAR[target], goal.col, goal.row,
             cal.latency_ms, cal.speed_gain, cal.turn_gain);
    commsSend(line);
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.printf("\n=== ROVER %d ===\n", ROVER_ID);
    hardwareInit();
    telemetryInit();
    strategyReset();
}

void loop() {
    // 1. Comandos (Serial o UDP)
    String cmd;
    if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
    if (commsPoll(cmd)) handleCommand(cmd);

    TelemetrySnapshot snap;
    telemetryGetSnapshot(snap);
    bool fresh = isTelemetryFresh();
    const char* state;

    // La ronda arranca en START_PHASE (READY en la versión final) y sigue en RUNNING
    bool round_on = snap.phase == PHASE_RUNNING ||
                    (snap.phase == PHASE_READY && START_PHASE == PHASE_READY);
    bool want_strategy = round_on || practice_run;

    if (test_until_ms > 0) {
        // 2. Prueba de motores: prioridad y caduca sola
        state = "PRUEBA";
        if (millis() > test_until_ms) stopAll();
    } else if (goto_active) {
        // 3. Práctica: ir a un punto
        state = "IR_A_PUNTO";
        if (!fresh) {
            motionStop();
            state = "SIN_TELEMETRIA";
        } else if (motionGoTo(motionPredict(snap), clampToField(goto_target, snap, FP_HALF_WIDTH), 1.0f, POWER_CRUISE, snap, COLOR_UNKNOWN) ||
                   millis() - goto_since_ms > GOTO_TIMEOUT_MS) {
            stopAll();
        }
    } else if (want_strategy) {
        // 4. Competencia: la estrategia corre a bordo
        if (!strategy_running) {
            strategyReset();
            strategy_running = true;
        }
        if (!fresh) {
            motionStop();                       // Nunca avanzar con una pose vieja
            state = "SIN_TELEMETRIA";
        } else {
            strategyStep(snap);
            state = strategyStateName();
        }
    } else {
        if (strategy_running) {
            strategy_running = false;
            motionStop();
        }
        state = fresh ? phaseName(snap.phase) : "SIN_TELEMETRIA";
    }

    // 5. Estado al cerebro y al Serial
    static uint32_t last_status_ms = 0;
    if (millis() - last_status_ms > STATUS_PERIOD_MS) {
        last_status_ms = millis();
        sendStatus(snap, fresh, state);
    }
    static uint32_t last_print_ms = 0;
    if (millis() - last_print_ms > 1000) {
        last_print_ms = millis();
        Serial.printf("[R%d] %s | enlace:%s fresca:%s | (%.1f, %.1f) %.0f° | plan %d | seq %u\n",
                      ROVER_ID, state, snap.is_connected ? "OK" : "NO", fresh ? "SI" : "NO",
                      snap.me.col, snap.me.row, snap.me.theta, planLoadedId(), (unsigned)snap.seq);
    }

    delay(LOOP_PERIOD_MS);
}
