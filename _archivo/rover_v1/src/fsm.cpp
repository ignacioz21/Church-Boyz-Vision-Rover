#include "../include/fsm.h"
#include "../include/config.h"
#include "../include/hardware.h"
#include "../include/telemetry.h"
#include "../include/navigation.h"
#include <math.h>

static RoverFsmState current_state = STATE_BOOT_INIT;
static RoverFsmState state_before_failsafe = STATE_IDLE;

static CubeColor target_color = COLOR_UNKNOWN;
static Vector2D target_waypoint;
static bool approaching_border_waypoint = false;
static uint32_t retreat_start_ms = 0;
static uint32_t unstuck_start_ms = 0;
static uint32_t last_telemetry_log_ms = 0;

static uint32_t stall_timer_ms = 0;
static Vector2D last_stall_pos(0, 0);
static bool ready_plan_calculated = false;

static const char* state_names[] = {
    "BOOT_INIT",
    "HEALTH_CHECK",
    "IDLE",
    "READY_PLAN",
    "NAV_PREAPPROACH",
    "PUSH_TO_DEPOT",
    "SAFE_RETREAT",
    "CHECK_NEXT_CUBE",
    "MISSION_FINISHED",
    "FAILSAFE_LAG",
    "UNSTUCK_REVERSE"
};

const char* fsmGetStateName(RoverFsmState state) {
    if (state >= 0 && state <= STATE_UNSTUCK_REVERSE) {
        return state_names[state];
    }
    return "UNKNOWN";
}

RoverFsmState fsmGetCurrentState() {
    return current_state;
}

static void transitionTo(RoverFsmState next) {
    if (next != current_state) {
        telemetrySendLog("[Rover V%d] Transicion: %s -> %s",
            ROVER_ID, fsmGetStateName(current_state), fsmGetStateName(next));
        current_state = next;
        setLedForState(current_state);
        
        if (next == STATE_NAV_PREAPPROACH) {
            stall_timer_ms = millis();
        }
    }
}

// Busca el siguiente cubo más cercano que no esté aún en su depósito
static CubeColor selectNextTargetCube(const TelemetrySnapshot &snapshot) {
    Vector2D my_pos(snapshot.my_rover.x, snapshot.my_rover.y);
    float min_dist = 9999.0f;
    CubeColor selected = COLOR_UNKNOWN;

    for (int i = 0; i < 3; i++) {
        const CubeData &cube = snapshot.cubes[i];
        if (!cube.detected) continue;

        Vector2D c_pos(cube.x, cube.y);
        Vector2D d_pos(snapshot.depots[cube.color].x, snapshot.depots[cube.color].y);

        // Si ya está en su depósito, ignorar
        if (isCubeInDepot(c_pos, d_pos, DEPOT_DELIVERY_MARGIN)) {
            continue;
        }

        float dist = euclideanDistance(my_pos, c_pos);
        if (dist < min_dist) {
            min_dist = dist;
            selected = cube.color;
        }
    }

    return selected;
}

void fsmInit() {
    current_state = STATE_BOOT_INIT;
    stopMotors();
    setLedForState(current_state);
    Serial.println("[FSM] Máquina de estados inicializada");
}

void fsmUpdate() {
    TelemetrySnapshot snapshot;
    bool has_data = telemetryGetSnapshot(snapshot);

    // Logging periódico cada 500 ms hacia la consola local y por TCP al Servidor de Visión
    if (millis() - last_telemetry_log_ms > 500) {
        last_telemetry_log_ms = millis();
        if (has_data && snapshot.my_rover.detected) {
            float err_deg = 0.0f;
            if (current_state == STATE_NAV_PREAPPROACH) {
                float dist = euclideanDistance(Vector2D(snapshot.my_rover.x, snapshot.my_rover.y), target_waypoint);
                float heading_cube = vectorToAngleDeg(snapshot.depots[target_color].x - snapshot.cubes[target_color].x,
                                                      snapshot.depots[target_color].y - snapshot.cubes[target_color].y);
                float target_h = (dist < TOL_DIST_ARRIVED_CELLS && !approaching_border_waypoint)
                                ? heading_cube
                                : vectorToAngleDeg(target_waypoint.x - snapshot.my_rover.x, target_waypoint.y - snapshot.my_rover.y);
                err_deg = angleDifferenceDeg(target_h, snapshot.my_rover.heading);
                telemetrySendLog("[Rover V%d] %s | Pos:(%.1f,%.1f) Th:%.0f | Wp:(%.1f,%.1f) D:%.1f | Err:%.0f deg",
                    ROVER_ID, fsmGetStateName(current_state),
                    snapshot.my_rover.x, snapshot.my_rover.y, snapshot.my_rover.heading,
                    target_waypoint.x, target_waypoint.y, dist, err_deg);
            } else if (current_state == STATE_PUSH_TO_DEPOT) {
                Vector2D depot_pos(snapshot.depots[target_color].x, snapshot.depots[target_color].y);
                float heading_depot = vectorToAngleDeg(depot_pos.x - snapshot.my_rover.x, depot_pos.y - snapshot.my_rover.y);
                err_deg = angleDifferenceDeg(heading_depot, snapshot.my_rover.heading);
                telemetrySendLog("[Rover V%d] %s | Pos:(%.1f,%.1f) Th:%.0f | Depot:(%.1f,%.1f) | Err:%.0f deg",
                    ROVER_ID, fsmGetStateName(current_state),
                    snapshot.my_rover.x, snapshot.my_rover.y, snapshot.my_rover.heading,
                    depot_pos.x, depot_pos.y, err_deg);
            } else {
                telemetrySendLog("[Rover V%d] Estado: %s | Fase: %d | Pos:(%.1f,%.1f) Th:%.0f",
                    ROVER_ID, fsmGetStateName(current_state), snapshot.phase,
                    snapshot.my_rover.x, snapshot.my_rover.y, snapshot.my_rover.heading);
            }
        } else {
            telemetrySendLog("[Rover V%d] Estado: %s | Esperando deteccion de vision...",
                ROVER_ID, fsmGetStateName(current_state));
        }
    }

    // -------------------------------------------------------------------------
    // VIGILANCIA GLOBAL DE FASES OFICIALES (Preparación, En curso, Final, Abort)
    // -------------------------------------------------------------------------
    if (has_data) {
        // Si el reloj o la visión emite FINISHED, parar de inmediato en cualquier estado
        if (snapshot.phase == PHASE_FINISHED && current_state != STATE_MISSION_FINISHED) {
            telemetrySendLog("[Rover V%d] Fase FINISHED recibida -> Detencion inmediata por fin de reto", ROVER_ID);
            stopMotors();
            transitionTo(STATE_MISSION_FINISHED);
            return;
        }

        // Si se aborta o reinicia la ronda a IDLE, detener y esperar
        if (snapshot.phase == PHASE_IDLE && current_state != STATE_IDLE && 
            current_state != STATE_BOOT_INIT && current_state != STATE_HEALTH_CHECK) {
            telemetrySendLog("[Rover V%d] Ronda en IDLE -> Motores detenidos", ROVER_ID);
            stopMotors();
            ready_plan_calculated = false;
            transitionTo(STATE_IDLE);
            return;
        }
    }

    // -------------------------------------------------------------------------
    // VIGILANCIA GLOBAL DE FAILSAFE (Watchdog de latencia / pérdida de visión)
    // -------------------------------------------------------------------------
    if (current_state == STATE_NAV_PREAPPROACH || 
        current_state == STATE_PUSH_TO_DEPOT || 
        current_state == STATE_SAFE_RETREAT ||
        current_state == STATE_UNSTUCK_REVERSE) {
        
        if (!isTelemetryFresh()) {
            telemetrySendLog("[Rover V%d] FAILSAFE: Perdida de vision o timeout -> Frenando", ROVER_ID);
            state_before_failsafe = current_state;
            stopMotors();
            transitionTo(STATE_FAILSAFE_LAG);
            return;
        }
    }

    // -------------------------------------------------------------------------
    // MÁQUINA DE ESTADOS
    // -------------------------------------------------------------------------
    switch (current_state) {

        case STATE_BOOT_INIT:
            transitionTo(STATE_HEALTH_CHECK);
            break;

        case STATE_HEALTH_CHECK:
            stopMotors();
            delay(100);
            transitionTo(STATE_IDLE);
            break;

        case STATE_IDLE:
            stopMotors();

            // Transición si la visión emite READY o ya está en RUNNING
            if (has_data && (snapshot.phase == PHASE_READY || snapshot.phase == PHASE_RUNNING)) {
                telemetrySendLog("[Rover V%d] Fase activa detectada (%d) -> Planificando objetivo...", ROVER_ID, snapshot.phase);
                ready_plan_calculated = false;
                transitionTo(STATE_READY_PLAN);
            }
            break;

        case STATE_READY_PLAN:
            stopMotors();

            // Calcular objetivo una sola vez para evitar saturar la red de logs
            if (!ready_plan_calculated && has_data) {
                target_color = selectNextTargetCube(snapshot);
                if (target_color == COLOR_UNKNOWN) {
                    telemetrySendLog("[Rover V%d] No hay cubos pendientes -> Mision cumplida", ROVER_ID);
                    transitionTo(STATE_MISSION_FINISHED);
                    break;
                }

                const CubeData &cube = snapshot.cubes[target_color];
                Vector2D cube_pos(cube.x, cube.y);
                Vector2D depot_pos(snapshot.depots[target_color].x, snapshot.depots[target_color].y);

                if (isNearBorder(cube_pos, EDGE_RISK_CELLS)) {
                    target_waypoint = calculateSafeBorderWaypoint(cube_pos);
                    approaching_border_waypoint = true;
                    telemetrySendLog("[Rover V%d] Cubo %d cerca de borde -> Waypoint seguro: (%.1f, %.1f)",
                        ROVER_ID, target_color, target_waypoint.x, target_waypoint.y);
                } else {
                    target_waypoint = calculatePreApproach(cube_pos, depot_pos, DIST_PREAPPROACH_CELLS);
                    approaching_border_waypoint = false;
                    telemetrySendLog("[Rover V%d] Cubo %d asignado -> Pre-approach: (%.1f, %.1f)",
                        ROVER_ID, target_color, target_waypoint.x, target_waypoint.y);
                }
                ready_plan_calculated = true;
            }

            // Transición a ejecución al iniciar RUNNING
            if (snapshot.phase == PHASE_RUNNING) {
                telemetrySendLog("[Rover V%d] Fase RUNNING detectada -> Arrancando navegacion", ROVER_ID);
                ready_plan_calculated = false;
                last_stall_pos = Vector2D(snapshot.my_rover.x, snapshot.my_rover.y);
                stall_timer_ms = millis();
                transitionTo(STATE_NAV_PREAPPROACH);
            }
            break;

        case STATE_NAV_PREAPPROACH: {
            Vector2D my_pos(snapshot.my_rover.x, snapshot.my_rover.y);
            const CubeData &cube = snapshot.cubes[target_color];
            Vector2D cube_pos(cube.x, cube.y);
            Vector2D depot_pos(snapshot.depots[target_color].x, snapshot.depots[target_color].y);

            float dist_to_target = euclideanDistance(my_pos, target_waypoint);

            // 1. REFLEJO TÁCTICO: Antichoque con Sensor Ultrasónico (Regla 12.2.11)
            // Si hay pared/obstáculo a < 12 cm y todavía no estamos sobre el cubo objetivo
            float sonar_cm = readUltrasonicCm();
            if (sonar_cm > 0.0f && sonar_cm < OBSTACLE_DIST_STOP_CM && dist_to_target > 2.5f) {
                telemetrySendLog("[Rover V%d] ¡OBSTACULO/PARED a %.1f cm! Desatascando...", ROVER_ID, sonar_cm);
                unstuck_start_ms = millis();
                stopMotors();
                transitionTo(STATE_UNSTUCK_REVERSE);
                break;
            }

            // 2. WATCHDOG DE ESTANCAMIENTO FÍSICO: Ruedas girando sin avance
            if (millis() - stall_timer_ms > STALL_TIME_MS) {
                float dist_moved = euclideanDistance(my_pos, last_stall_pos);
                if (dist_moved < 0.6f) { // Menos de 1.2 cm en 2.5 segundos mientras avanzaba
                    telemetrySendLog("[Rover V%d] ¡ATASCO DETECTADO! Sin avance (mov=%.2f cel). Retrocediendo...", ROVER_ID, dist_moved);
                    unstuck_start_ms = millis();
                    stopMotors();
                    transitionTo(STATE_UNSTUCK_REVERSE);
                    break;
                }
                last_stall_pos = my_pos;
                stall_timer_ms = millis();
            }

            // 3. Si estábamos en un punto de borde intermedio y ya llegamos, pasar al pre-aproach final
            if (approaching_border_waypoint) {
                if (dist_to_target < TOL_DIST_ARRIVED_CELLS) {
                    telemetrySendLog("[Rover V%d] Waypoint de borde alcanzado -> Pre-aproximacion detras del cubo", ROVER_ID);
                    target_waypoint = calculatePreApproach(cube_pos, depot_pos, DIST_PREAPPROACH_CELLS);
                    approaching_border_waypoint = false;
                }
            }

            float heading_to_cube = vectorToAngleDeg(depot_pos.x - cube_pos.x, depot_pos.y - cube_pos.y);
            float ang_alignment = fabsf(angleDifferenceDeg(heading_to_cube, snapshot.my_rover.heading));

            // 4. Condición de llegada: cerca de pre-aproximación y orientado hacia el cubo/depósito
            if (!approaching_border_waypoint && dist_to_target < TOL_DIST_ARRIVED_CELLS && ang_alignment < TOL_ANGLE_ALIGNED_DEG) {
                telemetrySendLog("[Rover V%d] ¡Alineado detras del cubo! (D:%.1f, Err:%.1f deg) -> Iniciando empuje",
                                 ROVER_ID, dist_to_target, ang_alignment);
                transitionTo(STATE_PUSH_TO_DEPOT);
                break;
            }

            // 5. Control de rumbo: si ya estamos en posición, pivotar al cubo; si no, navegar al waypoint
            float target_heading;
            if (dist_to_target < TOL_DIST_ARRIVED_CELLS && !approaching_border_waypoint) {
                target_heading = heading_to_cube;
            } else {
                target_heading = vectorToAngleDeg(target_waypoint.x - my_pos.x, target_waypoint.y - my_pos.y);
            }

            float v_left, v_right;
            calculateSteeringMotors(snapshot.my_rover.heading, target_heading, SPEED_CRUISE, v_left, v_right);
            setMotors(v_left, v_right);
            break;
        }

        case STATE_UNSTUCK_REVERSE: {
            // Maniobra activa de desatasco: retroceder en curva para despejar la pared/obstáculo
            setMotors(-0.45f, -0.25f);

            if (millis() - unstuck_start_ms >= UNSTUCK_DURATION_MS) {
                stopMotors();
                telemetrySendLog("[Rover V%d] Desatasco completado -> Reanudando navegacion", ROVER_ID);
                stall_timer_ms = millis();
                last_stall_pos = Vector2D(snapshot.my_rover.x, snapshot.my_rover.y);
                transitionTo(STATE_NAV_PREAPPROACH);
            }
            break;
        }

        case STATE_PUSH_TO_DEPOT: {
            const CubeData &cube = snapshot.cubes[target_color];
            Vector2D cube_pos(cube.x, cube.y);
            Vector2D depot_pos(snapshot.depots[target_color].x, snapshot.depots[target_color].y);

            // 1. Verificar si el cubo ya entró al depósito
            if (isCubeInDepot(cube_pos, depot_pos, DEPOT_DELIVERY_MARGIN)) {
                telemetrySendLog("[Rover V%d] ¡Cubo entregado con exito en deposito! -> SAFE_RETREAT", ROVER_ID);
                stopMotors();
                retreat_start_ms = millis();
                transitionTo(STATE_SAFE_RETREAT);
                break;
            }

            // 2. Guiar empuje recto apuntando al centro del depósito
            Vector2D my_pos(snapshot.my_rover.x, snapshot.my_rover.y);
            float heading_to_depot = vectorToAngleDeg(depot_pos.x - my_pos.x, depot_pos.y - my_pos.y);

            float v_left, v_right;
            calculateSteeringMotors(snapshot.my_rover.heading, heading_to_depot, SPEED_PUSH, v_left, v_right);
            setMotors(v_left, v_right);
            break;
        }

        case STATE_SAFE_RETREAT: {
            // Maniobra en curva hacia atrás para despejar las aletas del cubo
            setMotors(SPEED_RETREAT_L, SPEED_RETREAT_R);

            if (millis() - retreat_start_ms >= RETREAT_DURATION_MS) {
                stopMotors();
                telemetrySendLog("[Rover V%d] Safe Retreat completado -> Evaluando siguiente cubo", ROVER_ID);
                transitionTo(STATE_CHECK_NEXT_CUBE);
            }
            break;
        }

        case STATE_CHECK_NEXT_CUBE: {
            stopMotors();

            // Buscar si queda algún cubo pendiente
            target_color = selectNextTargetCube(snapshot);
            if (target_color != COLOR_UNKNOWN) {
                const CubeData &cube = snapshot.cubes[target_color];
                Vector2D cube_pos(cube.x, cube.y);
                Vector2D depot_pos(snapshot.depots[target_color].x, snapshot.depots[target_color].y);

                if (isNearBorder(cube_pos, EDGE_RISK_CELLS)) {
                    target_waypoint = calculateSafeBorderWaypoint(cube_pos);
                    approaching_border_waypoint = true;
                } else {
                    target_waypoint = calculatePreApproach(cube_pos, depot_pos, DIST_PREAPPROACH_CELLS);
                    approaching_border_waypoint = false;
                }

                telemetrySendLog("[Rover V%d] Siguiente cubo seleccionado: %d -> Pre-approach (%.1f, %.1f)",
                                 ROVER_ID, target_color, target_waypoint.x, target_waypoint.y);
                stall_timer_ms = millis();
                last_stall_pos = Vector2D(snapshot.my_rover.x, snapshot.my_rover.y);
                transitionTo(STATE_NAV_PREAPPROACH);
            } else {
                telemetrySendLog("[Rover V%d] ¡Todos los cubos han sido entregados! Finalizando mision.", ROVER_ID);
                transitionTo(STATE_MISSION_FINISHED);
            }
            break;
        }

        case STATE_MISSION_FINISHED:
            stopMotors();
            setLedForState(STATE_MISSION_FINISHED);
            break;

        case STATE_FAILSAFE_LAG:
            stopMotors();
            setLedForState(STATE_FAILSAFE_LAG);

            // Si la visión se restablece y los datos son frescos, reanudar
            if (isTelemetryFresh()) {
                telemetrySendLog("[Rover V%d] Telemetria recuperada. Reanudando estado: %s",
                                 ROVER_ID, fsmGetStateName(state_before_failsafe));
                transitionTo(state_before_failsafe);
            }
            break;
    }
}
