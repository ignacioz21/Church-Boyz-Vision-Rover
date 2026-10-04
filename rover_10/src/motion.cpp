#include "../include/motion.h"
#include "../include/config.h"
#include "../include/geometry.h"
#include "../include/hardware.h"
#include "../include/telemetry.h"

static MotionCal cal = { LOOP_LATENCY_MS, SPEED_GAIN, TURN_GAIN };
const MotionCal& motionCal() { return cal; }

// Historial de órdenes de motor (~1 s a 50 Hz) para integrar lo que el robot
// hizo desde el instante que describe la última pose de la cámara.
struct Cmd { uint32_t t_ms; float left, right; };
static const int HIST = 64;
static Cmd hist[HIST];
static int hist_head = 0;

void motionDrive(float left, float right) {
    hist[hist_head] = { millis(), left, right };
    hist_head = (hist_head + 1) % HIST;
    setMotors(left, right);
}

void motionStop() {
    motionDrive(0.0f, 0.0f);
}

Pose motionPredict(const TelemetrySnapshot &snap) {
    Pose pose;
    pose.p.col = snap.me.col;
    pose.p.row = snap.me.row;
    pose.theta = snap.me.theta;

    // La pose describe el instante (recepción - latencia): se integra desde ahí
    uint32_t now = millis();
    float horizon_ms = cal.latency_ms + (float)(now - snap.last_packet_time_ms) + snap.me.age_ms;
    horizon_ms = constrain(horizon_ms, 0.0f, 900.0f);
    uint32_t t_from = now - (uint32_t)horizon_ms;

    // El robot gira sobre el eje de las ruedas, que está detrás del marcador:
    // ese es el punto que se integra y el que se controla.
    Point axle = advance(pose.p, pose.theta, -AXLE_OFFSET);

    // Recorre el historial del más viejo al más nuevo; cada orden vale hasta la siguiente
    for (int i = 0; i < HIST; i++) {
        const Cmd &c = hist[(hist_head + i) % HIST];
        const Cmd &next = hist[(hist_head + i + 1) % HIST];
        uint32_t t_end = (i == HIST - 1) ? now : next.t_ms;
        if (c.t_ms == 0 || t_end <= t_from) continue;
        uint32_t t_start = max(c.t_ms, t_from);
        if (t_end <= t_start) continue;
        float dt = (t_end - t_start) / 1000.0f;

        float v = cal.speed_gain * (c.left + c.right) / 2.0f;       // celdas/s
        float w = cal.turn_gain * (c.right - c.left) / 2.0f;        // grados/s (antihorario +)
        axle = advance(axle, pose.theta + w * dt / 2.0f, v * dt);
        pose.theta += w * dt;
    }
    pose.p = axle;
    pose.theta = fmodf(pose.theta + 720.0f, 360.0f);
    return pose;
}

// Sentido del pivote en curso (+1 antihorario, -1 horario, 0 = ninguno). Se
// mantiene durante todo el giro para no cambiar de lado a mitad de camino.
static int turn_dir = 0;
static float turn_target = 0.0f;
static uint32_t last_turn_call_ms = 0;
static int escape_dir = 0;
static uint32_t escape_until_ms = 0;
static int pulse_dir = 0;
static uint32_t pulse_end_ms = 0, settle_end_ms = 0;

static bool pivot_gently = false;       // Con un cubo en las pinzas: siempre a la potencia mínima

static void pivot(int dir, float remaining_deg) {
    float power = constrain(remaining_deg * 0.006f, POWER_MIN_PIVOT, pivot_gently ? POWER_MIN_PIVOT : POWER_MAX_PIVOT);
    if (dir > 0) motionDrive(-power, power);        // Antihorario: derecha adelante
    else motionDrive(power, -power);
}

bool motionTurnTo(const Pose &pose, float target_heading, float tol_deg,
                  const TelemetrySnapshot &snap, CubeColor carried) {
    pivot_gently = carried != COLOR_UNKNOWN;
    uint32_t now = millis();
    // Si hace rato que no se llamaba, es un giro NUEVO: no se hereda el sentido del
    // anterior (podría ser la vuelta larga, con las pinzas por fuera de la cancha).
    if (now - last_turn_call_ms > 150) {
        turn_dir = 0;
        escape_until_ms = 0;
    }
    last_turn_call_ms = now;
    // Un pulso de ajuste fino en curso se termina siempre, y después se espera
    // quieto a que la cámara muestre el resultado. Si se decidiera con la pose
    // predicha, el pulso se cortaría antes de que las ruedas lleguen a moverse.
    if (now < pulse_end_ms) {
        pivot(pulse_dir, 0.0f);
        return false;
    }
    if (now < settle_end_ms) {
        motionDrive(0.0f, 0.0f);
        return false;
    }

    float diff = wrapDeg(target_heading - pose.theta);
    if (fabsf(diff) <= tol_deg) {
        turn_dir = 0;
        motionStop();
        return true;
    }

    // Giro nuevo (o el objetivo cambió): elegir de nuevo
    if (turn_dir != 0 && fabsf(wrapDeg(target_heading - turn_target)) > 20.0f) turn_dir = 0;
    turn_target = target_heading;

    int shortest = diff > 0 ? 1 : -1;

    // Ajuste fino (ya cerca del rumbo): por pulsos. Una corrección continua daría
    // toques tan cortos que las ruedas no vencen la fricción, y el robot oscilaría
    // creyendo que gira.
    if (fabsf(diff) < 15.0f && pivotClear(pose.p, pose.theta, target_heading, shortest, snap, carried)) {
        float rate = cal.turn_gain * POWER_MIN_PIVOT;                        // grados/s
        float pulse_ms = constrain(fabsf(diff) / rate * 1000.0f * 0.8f, 90.0f, 300.0f);
        turn_dir = shortest;
        pulse_dir = shortest;
        pulse_end_ms = now + (uint32_t)pulse_ms;
        settle_end_ms = pulse_end_ms + (uint32_t)cal.latency_ms + 120;
        pivot(pulse_dir, 0.0f);
        return false;
    }

    if (turn_dir == 0) {
        if (pivotClear(pose.p, pose.theta, target_heading, shortest, snap, carried)) turn_dir = shortest;
        else if (pivotClear(pose.p, pose.theta, target_heading, -shortest, snap, carried)) turn_dir = -shortest;
    }

    if (turn_dir == 0) {
        // Ningún lado es seguro (pegado a la línea o a un cubo): hacer sitio en
        // línea recta. Se mantiene el sentido elegido un rato, para no ir y venir.
        float creep = POWER_MIN_MOVE + 0.05f;
        if (now > escape_until_ms) {
            // Con holgura en las líneas: hacer sitio no debe terminar sacando al robot
            bool fwd_ok = poseClear(advance(pose.p, pose.theta, 1.5f), pose.theta, snap, carried, 0.2f, 0.5f);
            bool back_ok = poseClear(advance(pose.p, pose.theta, -1.5f), pose.theta, snap, carried, 0.2f, 0.5f);
            // Hacia atrás primero (aleja las pinzas de lo que tengan delante), salvo
            // con un cubo en las pinzas: retroceder lo dejaría atrás.
            if (carried != COLOR_UNKNOWN) back_ok = false;
            escape_dir = back_ok ? -1 : (fwd_ok ? 1 : 0);
            escape_until_ms = now + 1200;
        }
        if (escape_dir != 0 &&
            poseClear(advance(pose.p, pose.theta, escape_dir * 1.0f), pose.theta, snap, carried, 0.2f, 0.5f)) {
            motionDrive(escape_dir * creep, escape_dir * creep);
            return false;
        }
        // No hay cómo hacer sitio. Si el giro cabe justo (holgura mínima), se hace;
        // si no, no se gira a la fuerza: queda quieto y quien llama decide.
        escape_until_ms = 0;
        if (pivotClear(pose.p, pose.theta, target_heading, shortest, snap, carried, PIVOT_TIGHT)) turn_dir = shortest;
        else if (pivotClear(pose.p, pose.theta, target_heading, -shortest, snap, carried, PIVOT_TIGHT)) turn_dir = -shortest;
        else {
            motionDrive(0.0f, 0.0f);
            return false;
        }
    }

    float remaining = fmodf((target_heading - pose.theta) * turn_dir + 720.0f, 360.0f);
    pivot(turn_dir, remaining);
    return false;
}

bool motionGoTo(const Pose &pose, Point target, float tol, float max_power,
                const TelemetrySnapshot &snap, CubeColor carried) {
    float d = dist(pose.p, target);
    float heading = headingTo(pose.p, target);
    float diff = wrapDeg(heading - pose.theta);

    // Llegó, o se pasó por poco (el objetivo quedó detrás)
    if (d <= tol || (d <= 2.0f * tol && fabsf(diff) > 90.0f)) {
        turn_dir = 0;
        motionStop();
        return true;
    }

    // Desvío grande: pivotar (por el lado seguro). Una vez empezado el pivote se
    // termina hasta quedar bien orientado, para no alternar entre pivote y arco.
    if (fabsf(diff) > 60.0f || (turn_dir != 0 && fabsf(diff) > 15.0f)) {
        motionTurnTo(pose, heading, 10.0f, snap, carried);
        return false;
    }
    turn_dir = 0;

    // No avanzar hacia donde la huella tocaría algo o saldría de las líneas: ahí
    // se orienta primero hacia el objetivo (con sus propias comprobaciones).
    if (fabsf(diff) > 12.0f && !poseClear(advance(pose.p, pose.theta, 2.5f), pose.theta, snap, carried)) {
        motionTurnTo(pose, heading, 10.0f, snap, carried);
        return false;
    }

    // Avanza corrigiendo en arco. Más lento cuanto más desviado y cuanto más cerca:
    // lo que se avanza durante la latencia debe quedar por debajo de la tolerancia.
    float power = constrain(POWER_MIN_MOVE + d * 0.04f, POWER_MIN_MOVE, max_power);
    power = max(POWER_MIN_MOVE, power * cosf(diff * (float)M_PI / 180.0f));
    float steer = constrain(diff * 0.010f, -0.25f, 0.25f);
    motionDrive(power - steer, power + steer);
    return false;
}

void carryCommand(const Pose &pose, Point to, float cube_ahead, float &left, float &right) {
    Point cube = advance(pose.p, pose.theta, cube_ahead);
    float diff = wrapDeg(headingTo(pose.p, to) - pose.theta);
    // Lento al llegar: lo que se avanza durante la latencia debe ser poco
    float power = dist(cube, to) < 5.0f ? POWER_MIN_MOVE + 0.05f : POWER_PUSH;
    float steer = constrain(diff * 0.006f, -CARRY_STEER_MAX, CARRY_STEER_MAX);
    left = power - steer;
    right = power + steer;
}

// Espera hasta tener una pose fresca y quieta; devuelve false si no hay telemetría
static bool freshPose(TelemetrySnapshot &snap, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
        if (telemetryGetSnapshot(snap) && isTelemetryFresh()) return true;
        delay(10);
    }
    return false;
}

void motionCalibrate() {
    const float STEP_POWER = 0.40f, PIVOT_POWER = 0.38f;
    const uint32_t STEP_MS = 1000, PIVOT_MS = 700, SETTLE_MS = 800;
    TelemetrySnapshot s0, s;

    Serial.println("[CAL] Calibrando: avanza ~20 cm y gira. Deja espacio libre al frente.");
    if (!freshPose(s0, 2000)) { Serial.println("[CAL] Sin telemetria fresca. Abortado."); return; }

    // 1. Escalón de avance: latencia = tiempo hasta VER el primer movimiento
    Point p0 = { s0.me.col, s0.me.row };
    uint32_t t0 = millis();
    float latency = -1.0f;
    setMotors(STEP_POWER, STEP_POWER);
    while (millis() - t0 < STEP_MS) {
        telemetryGetSnapshot(s);
        Point p = { s.me.col, s.me.row };
        if (latency < 0 && dist(p0, p) > 0.4f) latency = millis() - t0;
        delay(5);
    }
    stopMotors();
    delay(SETTLE_MS);
    if (!freshPose(s, 2000)) { Serial.println("[CAL] Se perdio la telemetria. Abortado."); return; }
    Point p1 = { s.me.col, s.me.row };
    float moved = dist(p0, p1);

    // 2. Pivote: grados girados / tiempo
    float th0 = s.me.theta;
    setMotors(-PIVOT_POWER, PIVOT_POWER);
    delay(PIVOT_MS);
    stopMotors();
    delay(SETTLE_MS);
    if (!freshPose(s, 2000)) { Serial.println("[CAL] Se perdio la telemetria. Abortado."); return; }
    float turned = fabsf(wrapDeg(s.me.theta - th0));

    if (latency < 0 || moved < 1.0f) {
        Serial.printf("[CAL] El robot casi no avanzo (%.2f celdas). Revisar bateria/motores.\n", moved);
        return;
    }
    cal.latency_ms = latency;
    cal.speed_gain = moved / (STEP_MS / 1000.0f) / STEP_POWER;
    if (turned > 5.0f) cal.turn_gain = turned / (PIVOT_MS / 1000.0f) / PIVOT_POWER;

    Serial.printf("[CAL] Avanzo %.2f celdas, giro %.1f grados.\n", moved, turned);
    Serial.printf("[CAL] Copiar a config.h:\n"
                  "#define LOOP_LATENCY_MS         %.0f.0f\n"
                  "#define SPEED_GAIN              %.1ff\n"
                  "#define TURN_GAIN               %.0f.0f\n",
                  cal.latency_ms, cal.speed_gain, cal.turn_gain);
}
