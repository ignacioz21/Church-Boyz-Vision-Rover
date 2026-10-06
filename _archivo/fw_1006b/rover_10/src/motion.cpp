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

// Última vez que se le pidió movimiento a algún motor
static uint32_t last_motion_cmd_ms = 0;

// --- Giro trabado ----------------------------------------------------------------------
static uint32_t pivot_since_ms = 0;     // Desde cuándo los motores están pivotando (0 = no)
static uint32_t stall_since_ms = 0;     // Desde cuándo pivotan sin que el robot gire
static uint32_t halt_until_ms = 0;      // Detenido, avisando del obstáculo
static uint32_t bump_until_ms = 0;      // Después: corrimiento recto para salir de ese lugar
static Point obstacle_at;               // Dónde se trabó la última vez (eje)
static int bump_dir = 0;
static int stalled_pulses = 0;          // Pulsos de ajuste fino seguidos que no giraron nada
static float pulse_turned = 0.0f;       // Grados girados en el pulso en curso (giroscopio)
static uint32_t pulse_sample_ms = 0;
static uint32_t obstacle_until_ms = 0;
static uint16_t obstacle_count = 0;
// Solo se confía en el giroscopio después de verlo medir un giro real mandado por los
// motores. Si el sensor estuviera montado de otra forma (otro eje), nunca pasa a true
// y la detección queda apagada, en vez de dar falsos "trabado" todo el tiempo.
static bool gyro_trusted = false;

// Potencia extra que la regulación le está sumando al giro (largo / fino), para el monitor
static float pivot_trim = 0.0f;
static float fine_trim = 0.0f;
float motionPivotTrim() { return pivot_trim; }
float motionFineTrim() { return fine_trim; }

bool motionObstacle() { return millis() < obstacle_until_ms; }
Point motionObstaclePoint() { return obstacle_at; }
uint16_t motionObstacleCount() { return obstacle_count; }

// Se trabó. NO se fuerza el giro: se para, se avisa (estado OBSTACULO) y recién después
// se sale de ese punto con un corrimiento corto en línea recta, a la potencia normal,
// para intentar el giro desde otro lugar. Con un cubo en las pinzas el corrimiento es
// hacia adelante (atrás lo dejaría).
static void stallDetected(bool carrying, Point where) {
    uint32_t now = millis();
    obstacle_count++;
    obstacle_at = where;
    halt_until_ms = now + STALL_HALT_MS;
    // Después de avisar, sin cubo se corre un poco (atrás, y la vez siguiente adelante)
    // para probar el giro desde otro punto. Con un cubo en las pinzas NO se mueve:
    // empujaría el cubo fuera de lugar; solo para, avisa y vuelve a intentar.
    bump_dir = bump_dir == -1 ? 1 : -1;
    bump_until_ms = halt_until_ms + (carrying ? 0 : STALL_BUMP_MS);
    obstacle_until_ms = bump_until_ms;
    stall_since_ms = 0;
    pivot_since_ms = 0;
    stalled_pulses = 0;
    setMotors(0.0f, 0.0f);
    Serial.printf("[OBSTACULO] Giro trabado #%u en (%.1f, %.1f): paro; despues: %s\n",
                  (unsigned)obstacle_count, where.col, where.row, carrying ? "nada (llevo un cubo)" : bump_dir > 0 ? "adelante" : "atras");
}

// Giro continuo: ¿cuánto giró de verdad (giroscopio) desde que los motores pivotan?
// Se juzga por el ángulo acumulado en una ventana larga, no por la velocidad de un
// instante: un giro lento o que arranca a tirones no es un obstáculo.
static float window_turned = 0.0f;
static uint32_t window_sample_ms = 0;

static void stallCheck(bool carrying, Point where) {
    if (!imuReady() || pivot_since_ms == 0) { window_sample_ms = 0; window_turned = 0.0f; return; }
    uint32_t now = millis();
    float rate = fabsf(gyroZDps());
    if (rate > 40.0f) gyro_trusted = true;
    if (window_sample_ms != 0) window_turned += rate * (now - window_sample_ms) / 1000.0f;
    window_sample_ms = now;
    // Con un cubo en las pinzas el arranque es lento y la potencia se va regulando:
    // hay que darle más tiempo antes de decir que está trabado.
    if (now - pivot_since_ms < (uint32_t)(carrying ? STALL_WINDOW_CARRY_MS : STALL_WINDOW_MS)) return;
    // Fin de la ventana: si giró, se abre otra; si casi no giró, está trabado
    bool stalled = gyro_trusted && window_turned < STALL_MIN_DEG;
    pivot_since_ms = now;
    window_turned = 0.0f;
    if (stalled) stallDetected(carrying, where);
}

void motionDrive(float left, float right) {
    if (left != 0.0f || right != 0.0f) last_motion_cmd_ms = millis();
    bool pivoting = left * right < 0.0f && fabsf(left) > 0.15f && fabsf(right) > 0.15f;
    if (!pivoting) { pivot_since_ms = 0; stall_since_ms = 0; }
    else if (pivot_since_ms == 0) {
        pivot_since_ms = millis();
        window_turned = 0.0f;
        window_sample_ms = 0;
    }

    // Tope de velocidad de avance: por encima, la cámara pierde el marcador (sale
    // movido) y el robot se queda sin pose. Los pivotes (left = -right) no cambian.
    float v = fabsf(cal.speed_gain * (left + right) / 2.0f);
    if (v > MAX_SPEED) {
        left *= MAX_SPEED / v;
        right *= MAX_SPEED / v;
    }
    hist[hist_head] = { millis(), left, right };
    hist_head = (hist_head + 1) % HIST;
    setMotors(left, right);
}

void motionStop() {
    motionDrive(0.0f, 0.0f);
}

// Giro REAL medido por el giroscopio (~1,3 s a 50 Hz). Para saber cuánto giró el robot
// desde la última imagen se usa esto y no el modelo "potencia x ganancia": con un cubo
// en las pinzas, o sobre un piso que frena, el robot gira bastante menos de lo que el
// modelo supone, y creería haber llegado al rumbo mucho antes de tiempo.
struct GyroSample { uint32_t t_ms; float rate; };
static const int GYRO_HIST = 64;
static GyroSample gyro_hist[GYRO_HIST];
static int gyro_head = 0;
static uint32_t gyro_last_ms = 0;

static void gyroSample() {
    uint32_t now = millis();
    if (!imuReady() || now == gyro_last_ms) return;
    // Quieto desde hace rato: lo que marque el giroscopio es error de cero, se corrige.
    // (Si el robot se movió al encender, el cero inicial queda mal y todo giro se
    // mediría corrido.)
    if (now - last_motion_cmd_ms > 1200) gyroRezero();
    gyro_last_ms = now;
    gyro_hist[gyro_head] = { now, gyroZDps() * GYRO_SIGN };
    gyro_head = (gyro_head + 1) % GYRO_HIST;
}

// Grados girados entre t_from y t_to según el giroscopio; false si no hay datos que cubran el tramo
static bool gyroTurned(uint32_t t_from, uint32_t t_to, float &deg) {
    if (!imuReady()) return false;
    deg = 0.0f;
    uint32_t covered_from = 0;
    for (int i = 0; i < GYRO_HIST; i++) {
        const GyroSample &a = gyro_hist[(gyro_head + i) % GYRO_HIST];
        if (a.t_ms == 0) continue;
        if (covered_from == 0) covered_from = a.t_ms;
        // Cada muestra vale hasta la siguiente (o hasta ahora)
        uint32_t end = (i == GYRO_HIST - 1) ? t_to : gyro_hist[(gyro_head + i + 1) % GYRO_HIST].t_ms;
        if (end <= a.t_ms || end - a.t_ms > 200) end = a.t_ms + 20;     // Hueco: no inventar giro
        uint32_t lo = max(a.t_ms, t_from), hi = min(end, t_to);
        if (hi > lo) deg += a.rate * (hi - lo) / 1000.0f;
    }
    // Sirve si el historial arranca antes del tramo y está al día
    return covered_from != 0 && covered_from <= t_from + 40 && t_to - gyro_last_ms < 100;
}

Pose motionPredict(const TelemetrySnapshot &snap) {
    gyroSample();
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
        // El giro de este tramo: el medido por el giroscopio si lo hay; si no, el del modelo
        float turned;
        if (!gyroTurned(t_start, t_end, turned)) turned = w * dt;
        axle = advance(axle, pose.theta + turned / 2.0f, v * dt);
        pose.theta += turned;
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

// Ajuste fino: cuánto dura un pulso respecto a lo que dice el modelo. Se corrige solo
// según el resultado del pulso anterior (un robot que gira más rápido de lo calibrado
// se pasaría una y otra vez del rumbo, oscilando sin llegar a apuntar).
static float pulse_scale = 1.0f;
static float pulse_from_diff = 0.0f;    // Desvío que quiso corregir el último pulso (0 = ninguno)

static bool pivot_gently = false;       // Con un cubo en las pinzas: siempre a la potencia mínima

// Regulación del giro con el giroscopio. La potencia que hace falta para girar depende
// del robot (motores, baterías) y de la carga (un cubo en las pinzas): en vez de una
// potencia fija, se busca una VELOCIDAD de giro y se ajusta la potencia hasta lograrla.
// 'pivot_trim' es ese ajuste; se conserva de un giro al otro.
static uint32_t trim_sample_ms = 0;

static void pivotRegulate(float target_dps) {
    uint32_t now = millis();
    // Solo en giro sostenido (pasado el arranque) y con muestras seguidas
    if (!imuReady() || pivot_since_ms == 0 || now - pivot_since_ms < 200 || now - trim_sample_ms > 100) {
        trim_sample_ms = now;
        return;
    }
    float dt = (now - trim_sample_ms) / 1000.0f;
    trim_sample_ms = now;
    float error = target_dps - fabsf(gyroZDps());
    pivot_trim = constrain(pivot_trim + PIVOT_TRIM_GAIN * error * dt, 0.0f, PIVOT_TRIM_MAX);
}

// Ajuste fino con giroscopio (motionTurnTo): descanso después de cada corte, para no ir
// y venir alrededor del rumbo, y si ya quedó dentro de la tolerancia, se queda ahí.
static bool fine_holding = false;

// Un ajuste fino es UN movimiento medido: se parte de un rumbo confirmado por la cámara
// (robot quieto), se gira lo que falta contando los grados con el giroscopio, se corta
// y se espera quieto a que la cámara muestre dónde quedó. Recién ahí se decide si hace
// falta otro. Corregir sobre la marcha, sin esa espera, lo hace ir y venir.
static bool fine_active = false;        // Movimiento en curso
static int fine_dir = 0;
static float fine_delta = 0.0f;         // Grados que hay que girar en este movimiento
static float fine_turned = 0.0f;        // Grados girados hasta ahora (giroscopio)
static uint32_t fine_sample_ms = 0, fine_start_ms = 0;
static uint32_t fine_settle_until_ms = 0;

static void fineBegin(float diff) {
    fine_active = true;
    fine_dir = diff > 0 ? 1 : -1;
    fine_delta = fabsf(diff);
    fine_turned = 0.0f;
    fine_sample_ms = fine_start_ms = millis();
    fine_trim = 0.0f;
}

// Avanza el movimiento en curso o la espera que le sigue. true mientras esté ocupado.
static bool fineStep() {
    uint32_t now = millis();
    if (!fine_active) {
        if (now < fine_settle_until_ms) { motionDrive(0.0f, 0.0f); return true; }
        return false;
    }
    float rate = gyroZDps() * GYRO_SIGN * fine_dir;         // Positivo = gira hacia donde se pidió
    fine_turned += rate * (now - fine_sample_ms) / 1000.0f;
    fine_sample_ms = now;
    float left_deg = fine_delta - fine_turned;
    if (left_deg <= fabsf(rate) * FINE_COAST_S + 0.3f || now - fine_start_ms > FINE_MOVE_MAX_MS) {
        fine_active = false;
        fine_settle_until_ms = now + (uint32_t)cal.latency_ms + FINE_SETTLE_EXTRA_MS;
        motionDrive(0.0f, 0.0f);
        return true;
    }
    // Poca potencia para un retoque sin cubo, más con el cubo en las pinzas, y algo más
    // cuanto más falta. Si lleva un momento mandando y no arranca, sube de a poco.
    float power = (pivot_gently ? FINE_POWER_CARRY : FINE_POWER) + FINE_POWER_PER_DEG * left_deg;
    if (fabsf(rate) > 15.0f) fine_trim = 0.0f;
    else if (now - fine_start_ms > 150) fine_trim = min(fine_trim + FINE_KICK_STEP, pivot_gently ? FINE_KICK_MAX_CARRY : FINE_KICK_MAX);
    power = constrain(power + fine_trim, 0.18f, 0.65f);
    if (fine_dir > 0) motionDrive(-power, power);
    else motionDrive(power, -power);
    return true;
}

static void pivot(int dir, float remaining_deg) {
    float power = constrain(remaining_deg * 0.006f, POWER_MIN_PIVOT, pivot_gently ? POWER_MIN_PIVOT : POWER_MAX_PIVOT);
    // Velocidad que se quiere: lenta con un cubo; sin cubo, la que daría esa potencia
    // en un robot que responde bien
    // La potencia extra regulada es SOLO para girar con un cubo en las pinzas, que es
    // cuando hace falta fuerza. Sin cubo se gira con la potencia de siempre.
    if (pivot_gently) {
        if (remaining_deg > 0.0f) pivotRegulate(PIVOT_RATE_CARRY);
        power += pivot_trim;
    }
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
        pulse_from_diff = 0.0f;
        fine_holding = false;
        fine_active = false;
        fine_settle_until_ms = 0;
    }
    last_turn_call_ms = now;

    // Giro trabado: quieto mientras avisa; después, un corrimiento recto (si hay lugar)
    bool carrying = carried != COLOR_UNKNOWN;
    if (now < halt_until_ms) {
        motionDrive(0.0f, 0.0f);
        return false;
    }
    if (now < bump_until_ms) {
        float creep = POWER_MIN_MOVE + 0.08f;
        if (poseClear(advance(pose.p, pose.theta, bump_dir * 1.5f), pose.theta, snap, carried, 0.2f, 0.5f))
            motionDrive(bump_dir * creep, bump_dir * creep);
        else motionDrive(0.0f, 0.0f);
        return false;
    }
    stallCheck(carrying, pose.p);
    if (now < halt_until_ms) { motionDrive(0.0f, 0.0f); return false; }
    // Un pulso de ajuste fino en curso se termina siempre, y después se espera
    // quieto a que la cámara muestre el resultado. Si se decidiera con la pose
    // predicha, el pulso se cortaría antes de que las ruedas lleguen a moverse.
    // Cuánto gira de verdad con cada pulso, según el giroscopio. Se mide el pulso Y la
    // espera que le sigue: el robot gira sobre todo después de que termina la orden.
    if (now < settle_end_ms && imuReady()) {
        if (pulse_sample_ms != 0) pulse_turned += fabsf(gyroZDps()) * (now - pulse_sample_ms) / 1000.0f;
        pulse_sample_ms = now;
    }
    if (now < pulse_end_ms) {
        pivot(pulse_dir, 0.0f);
        return false;
    }
    if (now < settle_end_ms) {
        motionDrive(0.0f, 0.0f);
        return false;
    }
    if (pulse_sample_ms != 0) {
        // Terminó un pulso con su espera: tres seguidos sin girar casi nada = trabado
        pulse_sample_ms = 0;
        stalled_pulses = (imuReady() && gyro_trusted && pulse_turned < 0.7f) ? stalled_pulses + 1 : 0;
        pulse_turned = 0.0f;
        if (stalled_pulses >= 3) { stallDetected(carrying, pose.p); motionDrive(0.0f, 0.0f); return false; }
    }

    // Ajuste fino en curso (o su espera): no se decide nada hasta que termine
    if (fineStep()) return false;

    float diff = wrapDeg(target_heading - pose.theta);
    // Una vez dentro de la tolerancia se da por bueno con un margen extra: la cámara
    // baila un par de grados y sin ese margen se corregiría sin parar.
    if (fabsf(diff) <= tol_deg + (fine_holding ? FINE_HOLD_EXTRA : 0.0f)) {
        fine_holding = true;
        turn_dir = 0;
        motionStop();
        return true;
    }

    fine_holding = false;

    // Giro nuevo (o el objetivo cambió): elegir de nuevo
    if (turn_dir != 0 && fabsf(wrapDeg(target_heading - turn_target)) > 20.0f) turn_dir = 0;
    turn_target = target_heading;

    int shortest = diff > 0 ? 1 : -1;

    // Ajuste fino (ya cerca del rumbo): por pulsos. Una corrección continua daría
    // toques tan cortos que las ruedas no vencen la fricción, y el robot oscilaría
    // creyendo que gira.
    if (fabsf(diff) < 15.0f && pivotClear(pose.p, pose.theta, target_heading, shortest, snap, carried)) {
        // Con giroscopio: de corrido y despacio. El rumbo se conoce al instante (no hay
        // que esperar a la cámara en cada paso), así que se gira hasta llegar y se corta
        // un poco antes, según la velocidad que lleve, para que termine justo.
        if (imuReady() && gyro_trusted) {
            turn_dir = shortest;
            fineBegin(diff);
            fineStep();
            return false;
        }
        // Sin giroscopio: por pulsos, esperando a la cámara entre uno y otro
        // Resultado del pulso anterior: se pasó => más cortos; casi no giró => más largos
        if (pulse_from_diff != 0.0f) {
            if (diff * pulse_from_diff < 0.0f) pulse_scale *= 0.6f;
            else if (fabsf(diff) > 0.5f * fabsf(pulse_from_diff)) pulse_scale *= 1.4f;
            pulse_scale = constrain(pulse_scale, 0.3f, 2.0f);
        }
        pulse_from_diff = diff;
        float rate = cal.turn_gain * POWER_MIN_PIVOT;                        // grados/s
        // El pulso mínimo solo se acorta si este robot demostró que se pasa
        float shortest_ms = PULSE_MIN_MS * min(1.0f, pulse_scale);
        float pulse_ms = constrain(fabsf(diff) / rate * 1000.0f * 0.8f * pulse_scale, shortest_ms, 300.0f);
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
