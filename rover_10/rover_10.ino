/**
 * Rover 10 — firmware (Vision Rover Challenge)
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
#include "include/coord.h"

// La planificación anida varias funciones y además reporta por la red desde adentro:
// con la pila por defecto (8 KB) queda muy justo. Una pila desbordada reinicia la placa.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

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

// Por qué arrancó la placa: si se reinicia en plena ronda, esto dice si fue un bajón
// de voltaje (motores) o un fallo del programa. Se reporta al cerebro en "rst".
// Miga de pan: en qué parte del programa estaba la placa. Vive en una memoria que
// sobrevive a un reinicio por cuelgue o fallo, así el arranque siguiente puede decir
// dónde se quedó (sin necesidad de tener el cable conectado).
RTC_NOINIT_ATTR static uint32_t crumb_magic;
RTC_NOINIT_ATTR static uint32_t crumb_now;
static uint32_t crumb_previous = 0;         // Lo que dejó el arranque anterior (0 = nada)
#define CRUMB_MAGIC 0xC0FFEE10u
static inline void crumb(uint32_t where) { crumb_now = where; }

// Plan B si la placa se reinicia sola en plena ronda (cuelgue, fallo o bajón de
// voltaje): al volver a arrancar retoma por su cuenta, sin que nadie la toque.
//   - En una ronda oficial no hace falta guardar nada: la fase READY/RUNNING llega por
//     la telemetría y la estrategia arranca sola, igual que al principio.
//   - En una práctica, la orden de "iniciar" se guarda acá (memoria que sobrevive al
//     reinicio) para que la práctica siga.
// Lo que se pierde es la memoria de la ronda (intentos por cubo, capturas vetadas): la
// estrategia vuelve a mirar la cancha y sigue con lo que falte.
RTC_NOINIT_ATTR static uint32_t resume_practice;       // CRUMB_MAGIC = había una práctica en curso
RTC_NOINIT_ATTR static uint32_t resume_count;          // Veces que retomó desde que se encendió
RTC_NOINIT_ATTR static uint32_t reset_streak;          // Reinicios solos SEGUIDOS (se borra tras un rato andando bien)
static bool resumed_this_boot = false;
// El plan de la PC (con sus rutas) también sobrevive a un reinicio: se guarda el mensaje
// tal como llegó y se vuelve a cargar al arrancar. Lo que ya no sirva lo descarta la
// estrategia sola (cada ruta se comprueba contra la cancha antes de usarla).
#define PLAN_MSG_MAX 1024
RTC_NOINIT_ATTR static char plan_saved[PLAN_MSG_MAX];
RTC_NOINIT_ATTR static uint32_t plan_saved_magic;
static bool gyro_skipped = false;                      // Este arranque fue sin giroscopio por los reinicios
#define STREAK_CLEAR_MS 30000

static const char* resetName() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:  return "encendido";
        case ESP_RST_SW:       return "reinicio";
        case ESP_RST_PANIC:    return "fallo";
        case ESP_RST_INT_WDT:  return "colgado-int";       // Interrupciones bloqueadas
        case ESP_RST_TASK_WDT: return "colgado-tarea";     // Una tarea no soltó el procesador
        case ESP_RST_WDT:      return "colgado";
        case ESP_RST_BROWNOUT: return "voltaje";
        case ESP_RST_EXT:      return "boton";
        default:               return "otro";
    }
}

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
    resume_practice = 0;                // Parada pedida: no hay nada que retomar
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
            if (cmd.length() < PLAN_MSG_MAX) {
                strcpy(plan_saved, cmd.c_str());
                plan_saved_magic = CRUMB_MAGIC;
            }
            Serial.printf("[PLAN] Cargado (%d rutas): %s\n", planRouteCount(), cmd.c_str());
        } else {
            Serial.printf("[PLAN] Mal formado: %s\n", cmd.c_str());
        }
    } else if (c == 'V' && a > 0 && b > a) {    // Velocidad: "V,crucero,tope" (solo antes de la ronda)
        if (strategy_running) {
            Serial.println("[VEL] Ignorado: la ronda ya esta en curso");
        } else {
            motionSetSpeed(cmd.substring(a + 1, b).toFloat(), cmd.substring(b + 1).toFloat());
            Serial.printf("[VEL] Crucero %.2f, tope %.1f celdas/s\n", motionCruise(), motionMaxSpeed());
        }
    } else if (c == 'S') {                      // Práctica: correr la estrategia ya
        stopAll();
        practice_run = true;
        resume_practice = CRUMB_MAGIC;
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
        Serial.printf("[CMD] Desconocido: %s  (r | P | V,crucero,tope | S | G,col,row | L | f | A | A,r | M,izq,der)\n", cmd.c_str());
    }
}

static void sendStatus(const TelemetrySnapshot &snap, bool fresh, const char* state) {
    static const char COLOR_CHAR[] = { 'r', 'g', 'b' };
    CubeColor target = strategy_running ? strategyTarget() : COLOR_UNKNOWN;
    Point goal = goto_active ? goto_target : strategyGoal();
    const MotionCal &cal = motionCal();
    const StrategyStats &k = strategyStats();
    // "st": entregas, apartados, ayudas, veces que se corrió, e incidentes (todo lo que salió mal)
    int incidents = k.no_progress + k.nav_fail + k.aim_timeout + k.capture_timeout + k.no_drop_route +
                    k.carry_fail + k.cube_lost + k.drop_blocked + k.verify_fail;
    // "route": lo que piensa recorrer (solo mientras corre la estrategia)
    Point route[12];
    int route_n = strategy_running ? strategyRoute(route, 12) : 0;
    char route_txt[200] = "";
    int used = 0;
    for (int i = 0; i < route_n && used < (int)sizeof(route_txt) - 16; i++) {
        used += snprintf(route_txt + used, sizeof(route_txt) - used, "%s[%.1f,%.1f]", i ? "," : "", route[i].col, route[i].row);
    }
    uint32_t plan_n = 0, plan_ms = 0;
    navPlanStats(&plan_n, &plan_ms);
    float mot_l = 0.0f, mot_r = 0.0f;
    motionLastCommand(&mot_l, &mot_r);
    char line[960];
    snprintf(line, sizeof(line),
             "{\"id\":%d,\"state\":\"%s\",\"fresh\":%s,\"link\":%s,\"phase\":\"%s\","
             "\"col\":%.2f,\"row\":%.2f,\"theta\":%.1f,\"seq\":%u,"
             "\"plan\":%d,\"task\":\"%c\",\"gc\":%.1f,\"gr\":%.1f,"
             "\"lat\":%.0f,\"vg\":%.1f,\"wg\":%.0f,"
             "\"peer\":%s,\"st\":[%d,%d,%d,%d,%d],\"rst\":\"%s\",\"up\":%lu,\"imu\":%s,\"gz\":%.0f,\"ob\":%u,\"obp\":[%.1f,%.1f],\"trim\":[%.2f,%.2f],\"crumb\":%u,\"stk\":%u,\"fw\":\"" FW_VERSION "\",\"res\":%u,\"gf\":%u,\"rt\":%d,\"np\":[%u,%u],\"spd\":[%.2f,%.1f],\"gs\":%d,\"tm\":%d,\"as\":%d,\"mot\":[%.2f,%.2f],\"route\":[%s]}",
             ROVER_ID, state, fresh ? "true" : "false", snap.is_connected ? "true" : "false",
             phaseName(snap.phase), snap.me.col, snap.me.row, snap.me.theta, (unsigned)snap.seq,
             planLoadedId(), target == COLOR_UNKNOWN ? '-' : COLOR_CHAR[target], goal.col, goal.row,
             cal.latency_ms, cal.speed_gain, cal.turn_gain,
             coordPeer().heard ? "true" : "false", k.deliveries, k.parks, k.helps, k.clears, incidents,
             resetName(), (unsigned long)(millis() / 1000),
             imuReady() ? "true" : "false", gyroZDps(), (unsigned)motionObstacleCount(),
             motionObstaclePoint().col, motionObstaclePoint().row,
             motionPivotTrim(), motionFineTrim(),
             (unsigned)crumb_previous, (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned)resume_count, (unsigned)gyroFailures(),
             planRouteCount(), (unsigned)plan_n, (unsigned)plan_ms, motionCruise(), motionMaxSpeed(), gyro_skipped ? 1 : 0,
             motionTurnMode(), strategy_running && strategySeating() ? 1 : 0, mot_l, mot_r, route_txt);
    commsSend(line);
}

// Lo último que se reportó, para repetirlo mientras la estrategia está planificando
static TelemetrySnapshot last_snap;
static bool last_fresh = false;
static uint32_t last_status_ms = 0;

static void statusKeepAlive() {
    crumb(30);                              // Planificando
    if (millis() - last_status_ms < STATUS_PERIOD_MS) return;
    last_status_ms = millis();
    bool official = last_snap.phase == PHASE_READY || last_snap.phase == PHASE_RUNNING;
    if (!official || STATUS_DURING_ROUND) sendStatus(last_snap, last_fresh, "PLANIFICANDO");
}

void setup() {
    Serial.begin(115200);
    delay(300);
    crumb_previous = (crumb_magic == CRUMB_MAGIC && esp_reset_reason() != ESP_RST_POWERON) ? crumb_now : 0;
    // ¿Arranque normal (alguien lo encendió) o la placa se reinició sola?
    esp_reset_reason_t why = esp_reset_reason();
    bool self_reset = crumb_magic == CRUMB_MAGIC &&
                      (why == ESP_RST_PANIC || why == ESP_RST_INT_WDT || why == ESP_RST_TASK_WDT ||
                       why == ESP_RST_WDT || why == ESP_RST_BROWNOUT);
    if (!self_reset) {
        resume_practice = 0;
        resume_count = 0;
        reset_streak = 0;
        plan_saved_magic = 0;
    } else {
        resume_count++;
        reset_streak++;
        resumed_this_boot = true;
        if (resume_practice == CRUMB_MAGIC) practice_run = true;       // La práctica sigue
    }
    crumb_magic = CRUMB_MAGIC;
    crumb(1);                               // Arranque: motores y sensores
    Serial.printf("\n=== ROVER %d === (arranque: %s, antes estaba en %u)\n", ROVER_ID, resetName(), (unsigned)crumb_previous);
    hardwareInit();
    // Los cuelgues vistos vienen del bus del giroscopio. Si la placa se colgó justo al
    // arrancarlo, o ya van dos reinicios seguidos, este arranque va sin giroscopio: mejor
    // apuntar un poco peor que quedarse reiniciando.
    gyro_skipped = self_reset && (reset_streak >= 2 || crumb_previous == 2);
    crumb(2);                               // Arranque: giroscopio
    if (gyro_skipped) Serial.println("[IMU] Salteado: la placa viene de reiniciarse sola");
    else hardwareImuInit();
    crumb(3);                               // Arranque: red y telemetria
    telemetryInit();
    crumb(4);
    if (self_reset && plan_saved_magic == CRUMB_MAGIC) {
        plan_saved[PLAN_MSG_MAX - 1] = '\0';
        if (planLoad(String(plan_saved))) Serial.printf("[PLAN B] Plan %d recuperado tras el reinicio\n", planLoadedId());
    }
    strategyReset();
    strategySetKeepAlive(statusKeepAlive);
    if (resumed_this_boot)
        Serial.printf("[PLAN B] La placa se reinicio sola (%s). %s\n", resetName(),
                      practice_run ? "Retomo la practica." : "Si hay ronda en curso, la retomo al recibir la fase.");
}

void loop() {
    crumb(10);                              // Inicio del ciclo
    if (reset_streak != 0 && millis() > STREAK_CLEAR_MS) reset_streak = 0;     // Ya anda bien
    TelemetrySnapshot snap;
    telemetryGetSnapshot(snap);
    bool fresh = isTelemetryFresh();
    const char* state;
    last_snap = snap;
    last_fresh = fresh;

    // La ronda arranca en START_PHASE (READY en la versión final) y sigue en RUNNING
    bool round_on = snap.phase == PHASE_RUNNING ||
                    (snap.phase == PHASE_READY && START_PHASE == PHASE_READY);

    // 1. Comandos (Serial o UDP). Desde READY el rover no obedece a NADIE de afuera
    // (reglamento 9.5, 11.2.6, 11.2.7): los comandos se leen para vaciar la cola y se
    // descartan, sin excepción (tampoco el STOP: en un intento solo detiene el juez).
    bool official_round = snap.phase == PHASE_READY || snap.phase == PHASE_RUNNING;
    String cmd;
    bool got_serial = Serial.available();
    String serial_cmd = got_serial ? Serial.readStringUntil('\n') : String();
    bool got_udp = commsPoll(cmd);
    if (official_round) {
        if (got_serial || got_udp) Serial.println("[CMD] Ignorado: ronda oficial en curso");
        // Lo que se estuviera probando al llegar READY se corta: la ronda manda
        if (test_until_ms > 0 || goto_active) { test_until_ms = 0; goto_active = false; motionStop(); }
        practice_run = false;       // Y termina cuando la visión lo diga, no antes ni después
        resume_practice = 0;
    } else {
        if (got_serial) handleCommand(serial_cmd);
        if (got_udp) handleCommand(cmd);
    }
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
            navPlanStatsReset();
            strategy_running = true;
        }
        if (!fresh) {
            motionStop();                       // Nunca avanzar con una pose vieja
            state = "SIN_TELEMETRIA";
        } else {
            crumb(20);                          // Estrategia
            strategyStep(snap);
            crumb(21);
            // El giro se trabó contra algo del piso: se avisa mientras lo resuelve
            state = motionObstacle() ? "OBSTACULO" : strategyStateName();
        }
    } else {
        if (strategy_running) {
            strategy_running = false;
            motionStop();
        }
        state = fresh ? phaseName(snap.phase) : "SIN_TELEMETRIA";
        // Antes de la ronda también le cuento al compañero dónde estoy: así el enlace
        // entre rovers se puede comprobar en IDLE, antes de arrancar.
        if (fresh) {
            Pose me = motionPredict(snap);
            coordPublish(me, ACT_IDLE, COLOR_UNKNOWN, false, false, me.p, me.p);
        }
    }

    // 5. Estado al cerebro y al Serial
    crumb(40);                              // Reporte de estado
    if (millis() - last_status_ms > STATUS_PERIOD_MS) {
        last_status_ms = millis();
        // Reporte al monitor: solo sale del rover, la PC no responde nada
        if (!official_round || STATUS_DURING_ROUND) sendStatus(snap, fresh, state);
    }
    static uint32_t last_print_ms = 0;
    if (millis() - last_print_ms > 1000) {
        last_print_ms = millis();
        Serial.printf("[R%d] %s | enlace:%s fresca:%s | (%.1f, %.1f) %.0f° | plan %d | seq %u\n",
                      ROVER_ID, state, snap.is_connected ? "OK" : "NO", fresh ? "SI" : "NO",
                      snap.me.col, snap.me.row, snap.me.theta, planLoadedId(), (unsigned)snap.seq);
    }

    crumb(50);                              // Espera entre ciclos
    delay(LOOP_PERIOD_MS);
}
