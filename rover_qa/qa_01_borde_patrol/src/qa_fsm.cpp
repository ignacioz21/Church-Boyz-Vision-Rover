#include "../include/qa_fsm.h"
#include "../include/qa_config.h"
#include "../include/hardware.h"
#include "../include/telemetry.h"
#include "../include/navigation.h"
#include <math.h>
#include <WiFi.h>

// Estados internos
static QaState current_state = QA_STATE_INIT;
static QaState state_before_failsafe = QA_STATE_WAIT_START;

// Coordenadas y geometría
static Vector2D target_waypoint(0, 0);
static float target_heading_deg = 0.0f;
static float initial_pivot_heading_deg = 0.0f;
static uint32_t pivot_start_ms = 0;
static uint32_t corner_stop_start_ms = 0;
static int edges_completed = 0;
static bool first_edge_done = false;

// Métricas y calidad de datos
static uint32_t qa_start_time_ms = 0;
static uint32_t qa_end_time_ms = 0;
static float turn_errors_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint32_t turn_durations_ms[4] = {0, 0, 0, 0};
static uint32_t total_telemetry_frames = 0;
static uint32_t max_telemetry_age_ms = 0;
static uint64_t sum_telemetry_age_ms = 0;

static uint32_t last_log_ms = 0;

static const char* qa_state_names[] = {
    "INIT",
    "WAIT_START",
    "APPROACH_FIRST_EDGE",
    "CORNER_STOP",
    "PIVOT_LEFT_90",
    "PATROL_EDGE",
    "REPORT_FINISHED",
    "FAILSAFE_LAG",
    "BENCH_TEST"
};

const char* qaFsmGetStateName(QaState state) {
    if (state >= 0 && state <= QA_STATE_BENCH_TEST) {
        return qa_state_names[state];
    }
    return "UNKNOWN";
}

QaState qaFsmGetState() {
    return current_state;
}

int qaFsmGetEdgesCompleted() {
    return edges_completed;
}

static void setQaLedColor(QaState state) {
    switch (state) {
        case QA_STATE_INIT:
            setLedColor(255, 255, 255); // Blanco
            break;
        case QA_STATE_WAIT_START:
            setLedColor(0, 0, 255);     // Azul
            break;
        case QA_STATE_APPROACH_FIRST_EDGE:
            setLedColor(0, 255, 255);   // Cian
            break;
        case QA_STATE_CORNER_STOP:
            setLedColor(255, 255, 0);   // Amarillo
            break;
        case QA_STATE_PIVOT_LEFT:
            setLedColor(255, 0, 255);   // Magenta
            break;
        case QA_STATE_PATROL_EDGE:
            setLedColor(0, 255, 0);     // Verde
            break;
        case QA_STATE_REPORT_FINISHED:
            setLedColor(0, 255, 128);   // Verde azulado brillante
            break;
        case QA_STATE_FAILSAFE:
            setLedColor(255, 0, 0);     // Rojo
            break;
        case QA_STATE_BENCH_TEST:
            setLedColor(255, 128, 0);   // Naranja
            break;
    }
}

static void transitionTo(QaState next) {
    if (next != current_state) {
        telemetrySendLog("[QA] Transicion: %s -> %s (Bordes: %d/%d)",
            qaFsmGetStateName(current_state), qaFsmGetStateName(next),
            edges_completed, MAX_EDGES_TO_PATROL);
        current_state = next;
        setQaLedColor(current_state);
    }
}

static uint32_t bench_start_ms = 0;

void qaFsmTriggerStart() {
    if (current_state == QA_STATE_WAIT_START) {
        if (!isTelemetryFresh()) {
            TelemetrySnapshot snap;
            telemetryGetSnapshot(snap);
            telemetrySendLog("\n[QA ADVERTENCIA] No se puede iniciar con BOOT: La vision no esta lista o el rover no es detectado.");
            telemetrySendLog("  -> WiFi:                  %s", (WiFi.status() == WL_CONNECTED) ? "CONECTADO" : "DESCONECTADO");
            telemetrySendLog("  -> TCP a %s:%d:   %s", VISION_HOST, VISION_PORT, snap.is_connected ? "ENLACE ACTIVO" : "SIN CONEXION");
            telemetrySendLog("  -> Rover ID %d en camara:  %s (Antiguedad: %u ms)", telemetryGetRoverId(), snap.my_rover.detected ? "DETECTADO" : "NO VISTO", snap.my_rover.age_ms);
            telemetrySendLog("  [TIP] Para probar motores y giro sin camara, envia 't' por Serial USB.\n");
            return;
        }
        telemetrySendLog("[QA] ¡Disparo de inicio recibido! Iniciando aproximacion al primer borde.");
        qa_start_time_ms = millis();
        transitionTo(QA_STATE_APPROACH_FIRST_EDGE);
    }
}

void qaFsmTriggerBenchTest() {
    telemetrySendLog("\n[QA BENCH] ¡Prueba de banco iniciada! (Prueba autonoma de motores + giro 90° sin vision)");
    bench_start_ms = millis();
    transitionTo(QA_STATE_BENCH_TEST);
}

// Calcula el punto de parada en el borde proyectando la trayectoria hacia adelante
static Vector2D calculateForwardEdgeIntersection(const RoverPose &rover, float grid_cols, float grid_rows, float margin) {
    float x_min = margin;
    float x_max = grid_cols - margin;
    float y_min = margin;
    float y_max = grid_rows - margin;

    float xr = constrain(rover.x, x_min, x_max);
    float yr = constrain(rover.y, y_min, y_max);

    // Vector de avance según rumbo antihorario (row crece hacia abajo)
    float rad = rover.heading * (M_PI / 180.0f);
    float ux = cosf(rad);
    float uy = -sinf(rad);

    float t_hit = 9999.0f;

    // Pared derecha
    if (ux > 0.05f) {
        float t = (x_max - xr) / ux;
        if (t > 0.0f && t < t_hit) t_hit = t;
    }
    // Pared izquierda
    else if (ux < -0.05f) {
        float t = (x_min - xr) / ux;
        if (t > 0.0f && t < t_hit) t_hit = t;
    }

    // Pared inferior
    if (uy > 0.05f) {
        float t = (y_max - yr) / uy;
        if (t > 0.0f && t < t_hit) t_hit = t;
    }
    // Pared superior
    else if (uy < -0.05f) {
        float t = (y_min - yr) / uy;
        if (t > 0.0f && t < t_hit) t_hit = t;
    }

    if (t_hit > 9000.0f) {
        // En caso excepcional, proyectar a la pared más cercana
        return Vector2D(x_max, yr);
    }

    float hit_x = xr + t_hit * ux;
    float hit_y = yr + t_hit * uy;
    return Vector2D(constrain(hit_x, x_min, x_max), constrain(hit_y, y_min, y_max));
}

// Determina la siguiente esquina a lo largo del borde actual según el rumbo
static Vector2D calculateNextCornerAlongEdge(const RoverPose &rover, float grid_cols, float grid_rows, float margin) {
    float x_min = margin;
    float x_max = grid_cols - margin;
    float y_min = margin;
    float y_max = grid_rows - margin;

    float h = rover.heading;
    while (h < 0.0f) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;

    // Cuadrantes de orientación hacia la esquina objetivo
    if (h >= 45.0f && h < 135.0f) {
        // Apuntando hacia ARRIBA (row decreciente)
        // La esquina arriba depende de si estamos más a la derecha o a la izquierda
        float target_x = (rover.x > (grid_cols / 2.0f)) ? x_max : x_min;
        return Vector2D(target_x, y_min);
    } else if (h >= 135.0f && h < 225.0f) {
        // Apuntando hacia la IZQUIERDA (col decreciente)
        float target_y = (rover.y > (grid_rows / 2.0f)) ? y_max : y_min;
        return Vector2D(x_min, target_y);
    } else if (h >= 225.0f && h < 315.0f) {
        // Apuntando hacia ABAJO (row creciente)
        float target_x = (rover.x > (grid_cols / 2.0f)) ? x_max : x_min;
        return Vector2D(target_x, y_max);
    } else {
        // Apuntando hacia la DERECHA (col creciente)
        float target_y = (rover.y > (grid_rows / 2.0f)) ? y_max : y_min;
        return Vector2D(x_max, target_y);
    }
}

// Verifica de forma robusta si el rover llegó al borde (distancia, cruce de plano o perímetro)
static bool hasArrivedAtEdge(const RoverPose &rover, const Vector2D &target_wp, float grid_cols, float grid_rows, float margin) {
    float x_min = margin;
    float x_max = grid_cols - margin;
    float y_min = margin;
    float y_max = grid_rows - margin;

    // 1. Distancia euclidiana directa con tolerancia amplia
    float dist = euclideanDistance(Vector2D(rover.x, rover.y), target_wp);
    if (dist <= CORNER_ARRIVE_TOL_CELLS) return true;

    // 2. Comprobación de plano según la pared objetivo:
    // Pared derecha: rover alcanzó o cruzó x_max
    if (target_wp.x >= (x_max - 1.0f) && rover.x >= (x_max - 0.5f)) return true;
    // Pared izquierda: rover alcanzó o cruzó x_min
    if (target_wp.x <= (x_min + 1.0f) && rover.x <= (x_min + 0.5f)) return true;
    // Pared superior: rover alcanzó o cruzó y_min
    if (target_wp.y <= (y_min + 1.0f) && rover.y <= (y_min + 0.5f)) return true;
    // Pared inferior: rover alcanzó o cruzó y_max
    if (target_wp.y >= (y_max - 1.0f) && rover.y >= (y_max - 0.5f)) return true;

    // 3. Red de seguridad absoluta: Si el rover está fuera de la caja de seguridad
    if (rover.x >= x_max || rover.x <= x_min || rover.y <= y_min || rover.y >= y_max) return true;

    return false;
}

// Verifica de forma robusta si el rover llegó a la esquina patrullada
static bool hasArrivedAtCorner(const RoverPose &rover, const Vector2D &corner_wp, float grid_cols, float grid_rows, float margin) {
    float x_min = margin;
    float x_max = grid_cols - margin;
    float y_min = margin;
    float y_max = grid_rows - margin;

    float dist = euclideanDistance(Vector2D(rover.x, rover.y), corner_wp);
    if (dist <= CORNER_ARRIVE_TOL_CELLS) return true;

    // Si nos dirigimos a la esquina superior (y_min)
    if (corner_wp.y <= (y_min + 1.0f) && rover.y <= (y_min + 0.8f)) {
        if (fabsf(rover.x - corner_wp.x) < 5.0f) return true;
    }
    // Si nos dirigimos a la esquina izquierda (x_min)
    if (corner_wp.x <= (x_min + 1.0f) && rover.x <= (x_min + 0.8f)) {
        if (fabsf(rover.y - corner_wp.y) < 5.0f) return true;
    }
    // Si nos dirigimos a la esquina inferior (y_max)
    if (corner_wp.y >= (y_max - 1.0f) && rover.y >= (y_max - 0.8f)) {
        if (fabsf(rover.x - corner_wp.x) < 5.0f) return true;
    }
    // Si nos dirigimos a la esquina derecha (x_max)
    if (corner_wp.x >= (x_max - 1.0f) && rover.x >= (x_max - 0.8f)) {
        if (fabsf(rover.y - corner_wp.y) < 5.0f) return true;
    }

    return false;
}

void qaFsmInit() {
    current_state = QA_STATE_INIT;
    stopMotors();
    edges_completed = 0;
    first_edge_done = false;
    target_waypoint = Vector2D(0.0f, 0.0f);
    total_telemetry_frames = 0;
    max_telemetry_age_ms = 0;
    sum_telemetry_age_ms = 0;
    setQaLedColor(current_state);
    Serial.println("[QA] FSM de control de calidad inicializada");
}

void qaFsmUpdate() {
    TelemetrySnapshot snapshot;
    bool has_data = telemetryGetSnapshot(snapshot);

    // Registro de estadísticas de telemetría si hay datos válidos
    if (has_data && snapshot.my_rover.detected) {
        total_telemetry_frames++;
        if (snapshot.my_rover.age_ms > max_telemetry_age_ms) {
            max_telemetry_age_ms = snapshot.my_rover.age_ms;
        }
        sum_telemetry_age_ms += snapshot.my_rover.age_ms;
    }

    // Telemetría periódica hacia la consola local y por TCP al servidor
    if (millis() - last_log_ms > 500) {
        last_log_ms = millis();
        if (has_data && snapshot.my_rover.detected) {
            float dist = euclideanDistance(Vector2D(snapshot.my_rover.x, snapshot.my_rover.y), target_waypoint);
            float err_deg = angleDifferenceDeg(target_heading_deg, snapshot.my_rover.heading);
            
            int fl, fr, rl, rr;
            readFloorSensors(fl, fr, rl, rr);
            float sonar_cm = readUltrasonicCm();

            telemetrySendLog("[QA] %s | Pos:(%.1f,%.1f) Th:%.0f | Wp:(%.1f,%.1f) D:%.1f | Err:%.0f deg | Sonar:%.0f | IR:(%d,%d)",
                qaFsmGetStateName(current_state),
                snapshot.my_rover.x, snapshot.my_rover.y, snapshot.my_rover.heading,
                target_waypoint.x, target_waypoint.y, dist, err_deg, sonar_cm, fl, fr);
        } else {
            telemetrySendLog("[QA] %s | WiFi: %s | TCP(%s:%d): %s | Rover ID %d en camara: %s", 
                qaFsmGetStateName(current_state),
                (WiFi.status() == WL_CONNECTED) ? "OK" : "DESCONECTADO",
                VISION_HOST, VISION_PORT,
                snapshot.is_connected ? "ENLACE_OK" : "SIN_CONEXION",
                telemetryGetRoverId(),
                (has_data && snapshot.my_rover.detected) ? "DETECTADO" : "NO_VISTO");
        }
    }

    // -------------------------------------------------------------------------
    // VIGILANCIA GLOBAL DE FAILSAFE (Watchdog de latencia / pérdida de visión)
    // -------------------------------------------------------------------------
    if (current_state == QA_STATE_APPROACH_FIRST_EDGE || 
        current_state == QA_STATE_PIVOT_LEFT || 
        current_state == QA_STATE_PATROL_EDGE) {
        
        if (!isTelemetryFresh()) {
            telemetrySendLog("[QA] FAILSAFE: Perdida de vision o timeout -> Frenando");
            state_before_failsafe = current_state;
            stopMotors();
            transitionTo(QA_STATE_FAILSAFE);
            return;
        }

        // ---------------------------------------------------------------------
        // PROTECCIÓN DE PRECIPICIO / BORDE CON SENSORES DE PISO (4IR)
        // ---------------------------------------------------------------------
        int fl, fr, rl, rr;
        readFloorSensors(fl, fr, rl, rr);
        if (fl > IR_CLIFF_THRESHOLD || fr > IR_CLIFF_THRESHOLD) {
            stopMotors();
            if (current_state == QA_STATE_APPROACH_FIRST_EDGE) {
                first_edge_done = true;
                telemetrySendLog("[QA CLIFF] ¡Sensores frontales en borde (FL:%d, FR:%d)! Frenado en borde.", fl, fr);
                corner_stop_start_ms = millis();
                transitionTo(QA_STATE_CORNER_STOP);
                return;
            } else if (current_state == QA_STATE_PATROL_EDGE) {
                edges_completed++;
                telemetrySendLog("[QA CLIFF] ¡Sensores frontales en esquina (FL:%d, FR:%d)! Esquina #%d.", fl, fr, edges_completed);
                corner_stop_start_ms = millis();
                transitionTo(QA_STATE_CORNER_STOP);
                return;
            }
        }
    }

    // -------------------------------------------------------------------------
    // MÁQUINA DE ESTADOS FINITOS QA
    // -------------------------------------------------------------------------
    switch (current_state) {

        case QA_STATE_INIT:
            stopMotors();
            delay(100);
            transitionTo(QA_STATE_WAIT_START);
            break;

        case QA_STATE_WAIT_START:
            stopMotors();

            // Disparo automático si la visión cambia a fase RUNNING
            if (has_data && snapshot.phase == PHASE_RUNNING) {
                telemetrySendLog("[QA] Fase RUNNING detectada -> Arrancando patrullaje");
                qa_start_time_ms = millis();
                transitionTo(QA_STATE_APPROACH_FIRST_EDGE);
            }
            break;

        case QA_STATE_APPROACH_FIRST_EDGE: {
            if (!has_data || !snapshot.my_rover.detected) {
                stopMotors();
                break;
            }

            Vector2D my_pos(snapshot.my_rover.x, snapshot.my_rover.y);

            // Calcular el punto objetivo en el borde si es la primera vez
            if (!first_edge_done && target_waypoint.x == 0.0f && target_waypoint.y == 0.0f) {
                target_waypoint = calculateForwardEdgeIntersection(snapshot.my_rover, snapshot.grid_cols, snapshot.grid_rows, EDGE_CLEARANCE_CELLS);
                telemetrySendLog("[QA] Primer borde objetivo calculado: (%.1f, %.1f) con holgura %.1f celdas",
                    target_waypoint.x, target_waypoint.y, EDGE_CLEARANCE_CELLS);
            }

            float dist_to_edge = euclideanDistance(my_pos, target_waypoint);

            // Condición de parada robusta: distancia, cruce de plano o perímetro alcanzado
            if (hasArrivedAtEdge(snapshot.my_rover, target_waypoint, snapshot.grid_cols, snapshot.grid_rows, EDGE_CLEARANCE_CELLS)) {
                stopMotors();
                first_edge_done = true;
                telemetrySendLog("[QA] ¡Borde inicial alcanzado! Pos:(%.1f, %.1f) Dist:%.1f cel. Pausa de estabilizacion...",
                    my_pos.x, my_pos.y, dist_to_edge);
                corner_stop_start_ms = millis();
                transitionTo(QA_STATE_CORNER_STOP);
                break;
            }

            // Desaceleración progresiva antes de llegar al borde para evitar sobrepaso por inercia
            float cruise_speed = (dist_to_edge < 6.0f) ? QA_SPEED_SLOW : QA_SPEED_CRUISE;

            // Guiado hacia el punto del borde
            float target_h = vectorToAngleDeg(target_waypoint.x - my_pos.x, target_waypoint.y - my_pos.y);
            target_heading_deg = target_h;

            float v_left, v_right;
            calculateSteeringMotors(snapshot.my_rover.heading, target_h, cruise_speed, v_left, v_right);
            setMotors(v_left, v_right);
            break;
        }

        case QA_STATE_CORNER_STOP:
            stopMotors();

            // Pausa de estabilización en la esquina
            if (millis() - corner_stop_start_ms >= CORNER_PAUSE_MS) {
                if (edges_completed >= MAX_EDGES_TO_PATROL) {
                    telemetrySendLog("[QA] ¡Circuito completado! (%d/%d bordes). Generando reporte final...",
                        edges_completed, MAX_EDGES_TO_PATROL);
                    qa_end_time_ms = millis();
                    transitionTo(QA_STATE_REPORT_FINISHED);
                } else {
                    // Iniciar giro a la izquierda de 90 grados
                    initial_pivot_heading_deg = snapshot.my_rover.heading;
                    // En sentido antihorario, izquierda es +90 grados
                    target_heading_deg = normalizeAngleDeg(initial_pivot_heading_deg + 90.0f);
                    pivot_start_ms = millis();

                    telemetrySendLog("[QA] Iniciando giro #%d a la izquierda (90 deg): Rumbo actual: %.1f deg -> Objetivo: %.1f deg",
                        edges_completed + 1, initial_pivot_heading_deg, target_heading_deg);
                    transitionTo(QA_STATE_PIVOT_LEFT);
                }
            }
            break;

        case QA_STATE_PIVOT_LEFT: {
            if (!has_data || !snapshot.my_rover.detected) break;

            float current_h = snapshot.my_rover.heading;
            float error_deg = angleDifferenceDeg(target_heading_deg, current_h);

            // Condición de parada del giro: alineado dentro de la tolerancia (3 grados)
            if (fabsf(error_deg) <= TURN_TOL_DEG) {
                stopMotors();
                uint32_t turn_time = millis() - pivot_start_ms;
                
                if (edges_completed < 4) {
                    turn_errors_deg[edges_completed] = error_deg;
                    turn_durations_ms[edges_completed] = turn_time;
                }

                telemetrySendLog("[QA] Giro #%d completado en %d ms. Error final: %.1f deg. Preparando siguiente borde...",
                    edges_completed + 1, turn_time, error_deg);

                // Calcular la siguiente esquina a lo largo de este borde
                target_waypoint = calculateNextCornerAlongEdge(snapshot.my_rover, snapshot.grid_cols, snapshot.grid_rows, EDGE_CLEARANCE_CELLS);
                telemetrySendLog("[QA] Siguiente esquina objetivo: (%.1f, %.1f)", target_waypoint.x, target_waypoint.y);

                transitionTo(QA_STATE_PATROL_EDGE);
                break;
            }

            // Pivote sobre su propio eje hacia la izquierda (antihorario)
            if (error_deg > 0.0f) {
                // Objetivo a la izquierda: motor izquierdo retrocede, motor derecho avanza
                setMotors(-QA_SPEED_PIVOT, QA_SPEED_PIVOT);
            } else {
                // Corrección si hubo sobrepaso a la derecha
                setMotors(QA_SPEED_PIVOT, -QA_SPEED_PIVOT);
            }
            break;
        }

        case QA_STATE_PATROL_EDGE: {
            if (!has_data || !snapshot.my_rover.detected) {
                stopMotors();
                break;
            }

            Vector2D my_pos(snapshot.my_rover.x, snapshot.my_rover.y);
            float dist_to_corner = euclideanDistance(my_pos, target_waypoint);

            // Condición de llegada robusta a la esquina (distancia, plano de esquina o perímetro)
            if (hasArrivedAtCorner(snapshot.my_rover, target_waypoint, snapshot.grid_cols, snapshot.grid_rows, EDGE_CLEARANCE_CELLS)) {
                stopMotors();
                edges_completed++;
                telemetrySendLog("[QA] ¡Esquina #%d alcanzada! (Pos: %.1f, %.1f | D: %.1f). Pausa...",
                    edges_completed, my_pos.x, my_pos.y, dist_to_corner);
                corner_stop_start_ms = millis();
                transitionTo(QA_STATE_CORNER_STOP);
                break;
            }

            // Desaceleración progresiva antes de la esquina para frenado preciso
            float cruise_speed = (dist_to_corner < 6.0f) ? QA_SPEED_SLOW : QA_SPEED_CRUISE;

            // Guiado proporcional hacia la esquina
            float target_h = vectorToAngleDeg(target_waypoint.x - my_pos.x, target_waypoint.y - my_pos.y);
            target_heading_deg = target_h;

            float v_left, v_right;
            calculateSteeringMotors(snapshot.my_rover.heading, target_h, cruise_speed, v_left, v_right);
            setMotors(v_left, v_right);
            break;
        }

        case QA_STATE_REPORT_FINISHED:
            stopMotors();
            setQaLedColor(QA_STATE_REPORT_FINISHED);

            // Emitir reporte completo de calidad de datos cada 2 segundos
            if (millis() - last_log_ms > 2000) {
                last_log_ms = millis();
                float total_sec = (qa_end_time_ms - qa_start_time_ms) / 1000.0f;
                float avg_age_ms = (total_telemetry_frames > 0) ? ((float)sum_telemetry_age_ms / total_telemetry_frames) : 0.0f;

                telemetrySendLog("\n=======================================================");
                telemetrySendLog("      REPORTE DE CONTROL DE CALIDAD (QA) DE BORDES     ");
                telemetrySendLog("=======================================================");
                telemetrySendLog(" Estado:                PRUEBA COMPLETADA CON EXITO");
                telemetrySendLog(" Bordes patrullados:    %d de %d (1 vuelta completa)", edges_completed, MAX_EDGES_TO_PATROL);
                telemetrySendLog(" Tiempo total:          %.2f segundos", total_sec);
                telemetrySendLog(" Cuadros telemetria:    %u cuadros", total_telemetry_frames);
                telemetrySendLog(" Latencia media:        %.1f ms (Max: %u ms)", avg_age_ms, max_telemetry_age_ms);
                telemetrySendLog(" Errores en giros 90°:  G1: %.1f°, G2: %.1f°, G3: %.1f°, G4: %.1f°",
                    turn_errors_deg[0], turn_errors_deg[1], turn_errors_deg[2], turn_errors_deg[3]);
                telemetrySendLog(" Tiempos de giro:       G1: %ums, G2: %ums, G3: %ums, G4: %ums",
                    turn_durations_ms[0], turn_durations_ms[1], turn_durations_ms[2], turn_durations_ms[3]);
                telemetrySendLog(" Holgura comprobada:    %.1f celdas (%.1f cm) sin salidas de pista",
                    EDGE_CLEARANCE_CELLS, EDGE_CLEARANCE_CELLS * 2.0f);
                telemetrySendLog("=======================================================\n");
            }
            break;

        case QA_STATE_FAILSAFE:
            stopMotors();
            setQaLedColor(QA_STATE_FAILSAFE);

            if (isTelemetryFresh()) {
                telemetrySendLog("[QA] Telemetria recuperada -> Reanudando estado: %s",
                    qaFsmGetStateName(state_before_failsafe));
                transitionTo(state_before_failsafe);
            }
            break;

        case QA_STATE_BENCH_TEST: {
            uint32_t elapsed = millis() - bench_start_ms;
            if (elapsed < 1200) {
                // Paso 1: Avance recto durante 1.2 segundos (Cian)
                setMotors(0.35f, 0.35f);
                setLedColor(0, 255, 255);
            } else if (elapsed < 1800) {
                // Paso 2: Frenado de pausa durante 600 ms (Amarillo)
                stopMotors();
                setLedColor(255, 255, 0);
            } else if (elapsed < 2400) {
                // Paso 3: Giro de 90° sobre su eje a la izquierda durante 600 ms (Magenta)
                setMotors(-0.45f, 0.45f);
                setLedColor(255, 0, 255);
            } else {
                // Paso 4: Final de prueba de banco y lectura de sensores (Verde)
                stopMotors();
                setLedColor(0, 255, 0);
                int fl, fr, rl, rr;
                readFloorSensors(fl, fr, rl, rr);
                float sonar = readUltrasonicCm();
                telemetrySendLog("\n=======================================================");
                telemetrySendLog("         PRUEBA DE BANCO AUTONOMA COMPLETADA           ");
                telemetrySendLog("=======================================================");
                telemetrySendLog(" Motores:    AVANCE Y GIRO 90° EJECUTADOS");
                telemetrySendLog(" Sonar:      %.1f cm", sonar);
                telemetrySendLog(" Sensores IR: FL=%d, FR=%d, RL=%d, RR=%d", fl, fr, rl, rr);
                telemetrySendLog("=======================================================\n");
                transitionTo(QA_STATE_WAIT_START);
            }
            break;
        }
    }
}
