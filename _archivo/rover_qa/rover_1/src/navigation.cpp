#include "../include/navigation.h"
#include "../include/config.h"
#include "../include/telemetry.h"
#include "../include/hardware.h"
#include <math.h>
#include <WiFiUdp.h>

static float current_pre_dist = 10.0f;
void setPreApproachDistance(float blocks) { current_pre_dist = blocks; }
float getPreApproachDistance() { return current_pre_dist; }

static WiFiUDP udpMap;
static bool udpInitialized = false;


// =============================================================================
// NAVEGACIÓN — Implementación
// =============================================================================

bool simulate_routes_mode = false;
float sim_r10_p1_x = -1, sim_r10_p1_y = -1;
float sim_r10_p2_x = -1, sim_r10_p2_y = -1;
float sim_r10_p3_x = -1, sim_r10_p3_y = -1;

float sim_r11_p1_x = -1, sim_r11_p1_y = -1;
float sim_r11_p2_x = -1, sim_r11_p2_y = -1;
float sim_r11_p3_x = -1, sim_r11_p3_y = -1;


float debug_target_x = -1.0f;
float debug_target_y = -1.0f;

// Variables globales para trazar la ruta
CubeColor active_hunt_color = COLOR_UNKNOWN;
float debug_path_p1_x = -1, debug_path_p1_y = -1;
float debug_path_p2_x = -1, debug_path_p2_y = -1;
float debug_path_p3_x = -1, debug_path_p3_y = -1;

static bool isOnLineSegment(float px, float py, float x1, float y1, float x2, float y2) {
    if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0) return false;
    float L2 = pow(x2 - x1, 2) + pow(y2 - y1, 2);
    if (L2 == 0.0f) return false;
    float t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / L2;
    t = max(0.0f, min(1.0f, t));
    float proj_x = x1 + t * (x2 - x1);
    float proj_y = y1 + t * (y2 - y1);
    return sqrt(pow(px - proj_x, 2) + pow(py - proj_y, 2)) <= 0.6f; // Tolerancia ajustada para diagonales finas
}

void test_border() {
    TelemetrySnapshot snap;
    if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) {
        Serial.println("[TEST] Error: No hay telemetria inicial para saber donde estamos.");
        return;
    }

    // 1. Determinar hacia qué pared está mirando basado en su ángulo
    float h = snap.my_rover.heading;
    float start_x = snap.my_rover.x;
    float start_y = snap.my_rover.y;
    float margin = EDGE_CLEARANCE_CELLS;
    
    float dist_cells = 0;
    int dir_x = 0; // Para la cuadricula virtual
    int dir_y = 0;
    
    if (h >= 315.0f || h < 45.0f) { // Mirando a la Derecha (+X)
        dist_cells = (snap.grid_cols - margin) - start_x;
        dir_x = 1;
    } else if (h >= 45.0f && h < 135.0f) { // Mirando Arriba (-Y)
        dist_cells = start_y - margin;
        dir_y = -1;
    } else if (h >= 135.0f && h < 225.0f) { // Mirando a la Izquierda (-X)
        dist_cells = start_x - margin;
        dir_x = -1;
    } else { // Mirando Abajo (+Y)
        dist_cells = (snap.grid_rows - margin) - start_y;
        dir_y = 1;
    }

    if (dist_cells <= 0) {
        Serial.println("[TEST] Ya estamos sobre o fuera del borde seguro. Abortando.");
        return;
    }

    int target_steps = round(dist_cells);
    Serial.printf("\n[TEST] Iniciando marcha. Pared al frente a %.1f celdas.\n", dist_cells);
    Serial.printf("[TEST] Meta a cumplir: %d pasos (transiciones de piso).\n", target_steps);

    // 2. Iniciar movimiento
    setMotors(QA_SPEED_CRUISE, QA_SPEED_CRUISE);

    // 3. Bucle de conteo
    int steps = 0;
    int fl, fr, rl, rr;
    readFloorSensors(fl, fr, rl, rr);
    
    // Estado inicial del piso (oscuro o claro) basado en sensor Frontal Izquierdo (fl)
    // Umbrales con histeresis para evitar rebotes por ruido
    bool is_dark = (fl > 2000); 

    uint32_t start_time = millis();
    
    while (steps < target_steps) {
        readFloorSensors(fl, fr, rl, rr);
        
        // Histeresis: solo cambiamos de estado si el valor es muy claro o muy oscuro
        bool currently_dark = is_dark;
        if (fl > 2500) currently_dark = true;
        else if (fl < 1500) currently_dark = false;

        if (currently_dark != is_dark) {
            steps++;
            is_dark = currently_dark;
            
            // Calculamos posición virtual a estima
            float virtual_x = start_x + (steps * dir_x);
            float virtual_y = start_y + (steps * dir_y);
            
            Serial.printf("[TEST] Paso %d/%d superado! (Color: %s) -> Pos. virtual estimada: (%.1f, %.1f)\n", 
                steps, target_steps, is_dark ? "NEGRO" : "BLANCO", virtual_x, virtual_y);
        }

        // Timeout de seguridad de 10 segundos para no chocar si falla el sensor
        if (millis() - start_time > 10000) {
            Serial.println("[TEST] Timeout de seguridad! El rover no llego a tiempo.");
            break;
        }

        delay(10); // Evitar saturar el procesador
    }

    stopMotors();
    Serial.printf("[TEST] Fin. Pasos contados: %d. Vehiculo detenido en el borde seguro.\n", steps);
}

// Verifica si hay un comando de abortar ('r')
bool checkAbort(WiFiUDP &udpCmd) {
    if (Serial.available() > 0 && Serial.read() == 'r') return true;
    if (udpCmd.parsePacket() && udpCmd.read() == 'r') {
        udpCmd.flush();
        return true;
    }
    return false;
}

struct Waypoint {
    float x;
    float y;
};

void patrol_border() {
    simulate_routes_mode = false;
    Serial.println("\n[PATROL] Planeando ruta global desde el inicio (Sistema de Waypoints)...");
    extern WiFiUDP udpCmd;
    
    TelemetrySnapshot snap;
    if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) {
        Serial.println("[PATROL] Sin telemetria inicial. Abortando.");
        return;
    }

    // 1. ELABORACIÓN DEL PLAN DE RUTA (Waypoints)
    // Se calculan las 4 coordenadas de las esquinas seguras de la cancha
    float margin = EDGE_CLEARANCE_CELLS;
    float x_min = margin;
    float x_max = snap.grid_cols - margin;
    float y_min = margin;
    float y_max = snap.grid_rows - margin;
    
    Waypoint route[4] = {
        {x_max, y_min}, // WP 0: Esquina Superior Derecha
        {x_max, y_max}, // WP 1: Esquina Inferior Derecha
        {x_min, y_max}, // WP 2: Esquina Inferior Izquierda
        {x_min, y_min}  // WP 3: Esquina Superior Izquierda
    };
    
    Serial.println("[PATROL] Ruta planeada:");
    for(int i=0; i<4; i++) {
        Serial.printf("  WP %d: (%.1f, %.1f)\n", i, route[i].x, route[i].y);
    }
    
    int target_wp = 0; // Empezamos siempre apuntando al WP 0 (para esta prueba)
    
    // 2. EJECUCIÓN DEL PLAN (Navegación Autónoma)
    enum State { TURN_TO_WP, DRIVE_TO_WP };
    State state = TURN_TO_WP;
    
    while (true) {
        if (checkAbort(udpCmd)) break; // Stop de emergencia remoto
        
        if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) {
            stopMotors();
            delay(50);
            continue;
        }
        
        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;
        
        // ---------------------------------------------------------
        // SISTEMA DE INTERSECCION Y TRAFICO
        // ---------------------------------------------------------
        if (snap.peer_rover.detected) {
            float peer_dist = sqrt(pow(x - snap.peer_rover.x, 2) + pow(y - snap.peer_rover.y, 2));
            
            // Si están a menos de 20 celdas (40cm) de distancia, el Rover 11 cede el paso
            if (peer_dist < 20.0f && telemetryGetRoverId() == 11) {
                stopMotors();
                Serial.println("[TRAFICO] Peligro de colision dinamico. Rover 11 cede el paso al Rover 10...");
                delay(100);
                continue; 
            }
        }
        
        Waypoint wp = route[target_wp];
        debug_target_x = wp.x;
        debug_target_y = wp.y;
        
        // Matemáticas Vectoriales: Distancia Euclidiana
        float dist = sqrt(pow(wp.x - x, 2) + pow(wp.y - y, 2));
        
        // Matemáticas Vectoriales: Ángulo hacia el objetivo
        // Nota: dy es negativo porque Y crece hacia abajo en la pantalla/grilla
        float target_h = atan2(-(wp.y - y), (wp.x - x)) * 180.0f / M_PI;
        if (target_h < 0) target_h += 360.0f;
        
        // Calcular Error Angular (Diferencia más corta)
        float diff = target_h - h;
        while (diff <= -180.0f) diff += 360.0f;
        while (diff > 180.0f) diff -= 360.0f;
        
        if (state == TURN_TO_WP) {
            // Girar en su propio eje hasta enrutarse
            if (fabs(diff) < 5.0f) {
                stopMotors();
                state = DRIVE_TO_WP;
                Serial.printf("[PATROL] Enfilado perfecto a WP %d. Arrancando crucero...\n", target_wp);
                delay(300);
            } else {
                // Control Proporcional para el giro sobre su eje (P-Controller)
                // Esto evita que oscile bruscamente (overshoot) por la latencia de la cámara
                float turn_speed = diff * 0.008f; 
                
                // Añadir fuerza mínima para vencer la fricción estática de los motores
                if (turn_speed > 0 && turn_speed < 0.18f) turn_speed = 0.18f;
                if (turn_speed < 0 && turn_speed > -0.18f) turn_speed = -0.18f;
                
                // Limitar a la velocidad máxima permitida
                turn_speed = constrain(turn_speed, -QA_SPEED_PIVOT, QA_SPEED_PIVOT);
                
                setMotors(-turn_speed, turn_speed); 
            }
        } else if (state == DRIVE_TO_WP) {
            // Avanzar manteniendo la línea recta (Control Proporcional Básico)
            if (dist < CORNER_ARRIVE_TOL_CELLS) { // Llegó a la coordenada
                stopMotors();
                Serial.printf("[PATROL] WP %d alcanzado en (%.1f, %.1f)!\n", target_wp, x, y);
                target_wp = (target_wp + 1) % 4; // Avanza el plan al siguiente WP
                state = TURN_TO_WP;
                delay(300);
            } else {
                // Si mientras avanza se desvía, corrige los motores sin detenerse
                float Kp = 0.008f; // Sensibilidad del volante
                float correction = diff * Kp; 
                
                // Limitar la corrección máxima para que no frene el coche bruscamente
                if (correction > 0.15f) correction = 0.15f;
                if (correction < -0.15f) correction = -0.15f;
                
                // Motor Izquierdo = Base - correccion | Motor Derecho = Base + correccion
                setMotors(QA_SPEED_CRUISE - correction, QA_SPEED_CRUISE + correction);
            }
        }
        
        delay(20); // Ciclo de control a 50Hz
    }
    
    debug_target_x = -1.0f;
    debug_target_y = -1.0f;
    stopMotors();
    Serial.println("[PATROL] Fin de ruta planificada.");
}


// --- CAMPOS DE POTENCIAL ARTIFICIAL (APF) ---
float applyPotentialFields(float cx, float cy, float tx, float ty, const TelemetrySnapshot &snap, CubeColor target_color) {
    float dx_att = tx - cx;
    float dy_att = ty - cy;
    float dist_att = sqrt(dx_att*dx_att + dy_att*dy_att);
    if (dist_att < 0.1f) return snap.my_rover.heading;
    
    // Fuerza Atractiva Normalizada
    float Fx = dx_att / dist_att;
    float Fy = dy_att / dist_att;
    
    float Fx_rep = 0;
    float Fy_rep = 0;
    
    // Radio de influencia: 8 bloques = 16 cm (15cm margen de seguridad + radio del rover)
    float R_rep = 8.5f; 
    float K_rep = 1.8f; // Magnitud de la repulsion
    
    auto add_repulsion = [&](float ox, float oy, float custom_r) {
        float dx = cx - ox;
        float dy = cy - oy;
        float dist = sqrt(dx*dx + dy*dy);
        if (dist > 0.1f && dist < custom_r) {
            float force = K_rep * (custom_r - dist) / dist;
            Fx_rep += (dx / dist) * force;
            Fy_rep += (dy / dist) * force;
        }
    };
    
    // 1. Evitar al otro rover (Es mas grande, margen mayor)
    if (snap.peer_rover.detected) {
        add_repulsion(snap.peer_rover.x, snap.peer_rover.y, 10.0f); // 20 cm
    }
    
    // 2. Evitar otros cubos estacionados
    for (int i=0; i<3; i++) {
        if (!snap.cubes[i].detected || snap.cubes[i].color == target_color) continue;
        // Evitar repelerse de los bordes del depot final
        float dist_to_target = sqrt(pow(tx - snap.cubes[i].x, 2) + pow(ty - snap.cubes[i].y, 2));
        // Reducir tolerancia para que SÍ esquive cubos que estan bloqueando el camino
        if (dist_to_target > 2.0f) {
            add_repulsion(snap.cubes[i].x, snap.cubes[i].y, R_rep);
        }
    }
    
    // Atenuar la repulsión a medida que nos acercamos al objetivo para no entrar en un bucle local
    if (dist_att < 12.0f) {
        float attenuation = dist_att / 12.0f;
        Fx_rep *= attenuation;
        Fy_rep *= attenuation;
    }
    
    Fx += Fx_rep;
    Fy += Fy_rep;
    
    float target_h = atan2(-Fy, Fx) * 180.0f / M_PI;
    if (target_h < 0) target_h += 360.0f;
    
    return target_h;
}

bool calculateAvoidanceWaypoint(float rx, float ry, float tx, float ty, const TelemetrySnapshot &snap, CubeColor target_color, float &out_wx, float &out_wy) {
    float path_dx = tx - rx;
    float path_dy = ty - ry;
    float path_len = sqrt(path_dx*path_dx + path_dy*path_dy);
    if (path_len < 1.0f) return false;
    
    float path_nx = path_dx / path_len;
    float path_ny = path_dy / path_len;

    bool collision = false;
    float closest_obst_x = 0;
    float closest_obst_y = 0;
    float min_dist_along = 999.0f;

    // Chequear al OTRO rover como obstaculo principal (mayor margen de seguridad)
    if (snap.peer_rover.detected) {
        float ox = snap.peer_rover.x;
        float oy = snap.peer_rover.y;
        
        float vx = ox - rx;
        float vy = oy - ry;
        
        float dist_along = vx * path_nx + vy * path_ny;
        if (dist_along > 0.0f && dist_along < path_len) {
            float proj_x = rx + dist_along * path_nx;
            float proj_y = ry + dist_along * path_ny;
            float dist_side = sqrt(pow(ox - proj_x, 2) + pow(oy - proj_y, 2));
            
            if (dist_side < 7.0f) { // Margen MUCHO mayor para el otro rover (chasis ancho)
                if (dist_along < min_dist_along) {
                    min_dist_along = dist_along;
                    closest_obst_x = ox;
                    closest_obst_y = oy;
                    collision = true;
                }
            }
        }
    }

    // Chequear los otros cubos
    for (int i = 0; i < 3; i++) {
        if (snap.cubes[i].color == target_color || !snap.cubes[i].detected) continue;
        
        float ox = snap.cubes[i].x;
        float oy = snap.cubes[i].y;
        
        float vx = ox - rx;
        float vy = oy - ry;
        
        float dist_along = vx * path_nx + vy * path_ny;
        if (dist_along > 0.0f && dist_along < path_len) {
            float proj_x = rx + dist_along * path_nx;
            float proj_y = ry + dist_along * path_ny;
            float dist_side = sqrt(pow(ox - proj_x, 2) + pow(oy - proj_y, 2));
            
            if (dist_side < 4.5f) { // Margen de seguridad aumentado
                if (dist_along < min_dist_along) {
                    min_dist_along = dist_along;
                    closest_obst_x = ox;
                    closest_obst_y = oy;
                    collision = true;
                }
            }
        }
    }
    
    if (collision) {
        // Smart Vector Deflection: Check both sides and pick the safest (closest to center)
        float side1_nx = -path_ny;
        float side1_ny = path_nx;
        float side2_nx = path_ny;
        float side2_ny = -path_nx;
        
        float base_x = rx + min_dist_along * path_nx;
        float base_y = ry + min_dist_along * path_ny;
        
        float wx1 = base_x + side1_nx * 6.0f;
        float wy1 = base_y + side1_ny * 6.0f;
        
        float wx2 = base_x + side2_nx * 6.0f;
        float wy2 = base_y + side2_ny * 6.0f;
        
        float mid_x = snap.grid_cols / 2.0f;
        float mid_y = snap.grid_rows / 2.0f;
        
        float dist1_to_center = sqrt(pow(wx1 - mid_x, 2) + pow(wy1 - mid_y, 2));
        float dist2_to_center = sqrt(pow(wx2 - mid_x, 2) + pow(wy2 - mid_y, 2));
        
        if (dist1_to_center < dist2_to_center) {
            out_wx = wx1; out_wy = wy1;
        } else {
            out_wx = wx2; out_wy = wy2;
        }
        
        out_wx = constrain(out_wx, 3.0f, snap.grid_cols - 3.0f);
        out_wy = constrain(out_wy, 3.0f, snap.grid_rows - 3.0f);
        return true;
    }
    return false;
}

CubeColor get_closest_pending_cube(const TelemetrySnapshot &snap, bool completed[3]) {
    CubeColor best_color = COLOR_UNKNOWN;
    float min_dist = 9999.0f;
    float rx = snap.my_rover.x;
    float ry = snap.my_rover.y;

    for (int i = 0; i < 3; i++) {
        if (completed[i]) continue; // Ignorar si está marcado como completado
        if (!snap.cubes[i].detected) continue;
        
        float cx = snap.cubes[i].x;
        float cy = snap.cubes[i].y;
        float dx = snap.depots[i].x;
        float dy = snap.depots[i].y;
        
        // Si no estaba completado pero ahora vemos que fisicamente esta en el deposito, lo completamos y evitamos
        if (sqrt(pow(cx - dx, 2) + pow(cy - dy, 2)) < 6.0f) {
            completed[i] = true;
            continue;
        }
        
        float dist_to_rover = sqrt(pow(cx - rx, 2) + pow(cy - ry, 2));
        if (dist_to_rover < min_dist) {
            min_dist = dist_to_rover;
            best_color = (CubeColor)i;
        }
    }
    return best_color;
}


void go_home(float target_x, float target_y) {
    Serial.printf("[HOME] Regresando a punto de salida exacto (%.2f, %.2f)...\n", target_x, target_y);
    unsigned long start_time = millis();
    while (millis() - start_time < 10000) { // Timeout de 10 seg
        TelemetrySnapshot snap;
        if (!telemetryGetSnapshot(snap)) {
            delay(50);
            continue;
        }
        
        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;
        
        float dx = target_x - x;
        float dy = target_y - y;
        float dist = sqrt(dx*dx + dy*dy);
        
        if (dist < 2.0f) {
            stopMotors();
            Serial.println("[HOME] Base alcanzada. Estacionado.");
            return;
        }
        
        float target_h = applyPotentialFields(x, y, target_x, target_y, snap, COLOR_UNKNOWN);
        float diff = target_h - h;
        while (diff <= -180.0f) diff += 360.0f;
        while (diff > 180.0f) diff -= 360.0f;
        
        if (fabs(diff) > 25.0f) {
            float turn_speed = diff * 0.008f;
            if (turn_speed > 0 && turn_speed < 0.16f) turn_speed = 0.16f;
            if (turn_speed < 0 && turn_speed > -0.16f) turn_speed = -0.16f;
            turn_speed = constrain(turn_speed, -0.3f, 0.3f);
            setMotors(-turn_speed, turn_speed);
        } else {
            float fwd = 0.4f;
            float corr = diff * 0.015f;
            corr = constrain(corr, -0.15f, 0.15f);
            setMotors(fwd - corr, fwd + corr);
        }
        delay(50);
    }
    stopMotors();
}

bool goto_step(float target_x, float target_y) {
    static uint32_t last_warn_ms = 0;
    TelemetrySnapshot snap;
    bool ok = telemetryGetSnapshot(snap);

    // Sin pose fresca no se avanza a ciegas
    if (!ok || !isTelemetryFresh()) {
        stopMotors();
        if (millis() - last_warn_ms > 1000) {
            last_warn_ms = millis();
            Serial.printf("[GOTO] Detenido: sin telemetria fresca (TCP:%s, rover %d visto:%s, age:%ums)\n",
                          snap.is_connected ? "OK" : "SIN_CONEXION", telemetryGetRoverId(),
                          snap.my_rover.detected ? "SI" : "NO", snap.my_rover.age_ms);
        }
        return false;
    }

    float x = snap.my_rover.x;
    float y = snap.my_rover.y;
    float h = snap.my_rover.heading;
    float dx = target_x - x;
    float dy = target_y - y;
    float dist = sqrt(dx*dx + dy*dy);

    if (dist < GOTO_ARRIVE_TOL_CELLS) {
        stopMotors();
        return true;
    }

    float target_h = applyPotentialFields(x, y, target_x, target_y, snap, COLOR_UNKNOWN);
    float diff = target_h - h;
    while (diff <= -180.0f) diff += 360.0f;
    while (diff > 180.0f) diff -= 360.0f;

    if (fabs(diff) > GOTO_PIVOT_THRESH_DEG) {
        float turn_speed = diff * 0.008f;
        if (turn_speed > 0 && turn_speed < GOTO_PIVOT_MIN) turn_speed = GOTO_PIVOT_MIN;
        if (turn_speed < 0 && turn_speed > -GOTO_PIVOT_MIN) turn_speed = -GOTO_PIVOT_MIN;
        turn_speed = constrain(turn_speed, -GOTO_PIVOT_MAX, GOTO_PIVOT_MAX);
        setMotors(-turn_speed, turn_speed);
    } else {
        float corr = constrain(diff * 0.015f, -0.2f, 0.2f);
        setMotors(GOTO_SPEED_FWD - corr, GOTO_SPEED_FWD + corr);
    }
    return false;
}

bool hunt_cube(CubeColor target_color) {
    Serial.printf("\n[HUNT] Iniciando mision: Buscar y depositar cubo %d...\n", target_color);
    extern WiFiUDP udpCmd;
    active_hunt_color = target_color;
    
    TelemetrySnapshot snap;
    if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) {
        Serial.println("[HUNT] Sin telemetria inicial. Abortando.");
        active_hunt_color = COLOR_UNKNOWN;
        return false;
    }

    float cube_x = -1, cube_y = -1;
    for(int i=0; i<3; i++) {
        if (snap.cubes[i].color == target_color) {
            cube_x = snap.cubes[i].x; cube_y = snap.cubes[i].y;
        }
    }
    float depot_x = snap.depots[target_color].x;
    float depot_y = snap.depots[target_color].y;

    if (cube_x < 0 || depot_x < 0) {
        Serial.println("[HUNT] Cubo o deposito no encontrados en la telemetria.");
        active_hunt_color = COLOR_UNKNOWN;
        return false;
    }

    float dx = cube_x - depot_x;
    float dy = cube_y - depot_y;
    float len = sqrt(dx*dx + dy*dy);
    if (len == 0.0f) len = 1.0f;
    
    float pre_x = cube_x + (dx/len) * current_pre_dist;
    float pre_y = cube_y + (dy/len) * current_pre_dist;
    pre_x = constrain(pre_x, 3.0f, snap.grid_cols - 3.0f);
    pre_y = constrain(pre_y, 3.0f, snap.grid_rows - 3.0f);

    float push_target_x = depot_x + (dx/len) * 4.0f;
    float push_target_y = depot_y + (dy/len) * 4.0f;

    enum HuntState { TURN_PRE, DRIVE_PRE, TURN_AVOID, DRIVE_AVOID, VERIFY_TELEMETRY_CUBE, TURN_APPROACH, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH, BACKUP_AWAY, VERIFY_DROP, BACKUP_RETRY };
    HuntState state = TURN_PRE;
    HuntState last_state = TURN_PRE;
    unsigned long wait_start_ms = 0;
    unsigned long mission_start_ms = millis();
    float avoid_x = 0, avoid_y = 0;
    unsigned long yield_start_ms = 0;
    bool is_yielding = false;
    bool ignore_traffic = false;
    
    setLedColor(255, 128, 0); 
    bool success = false;

    while (true) {
        float current_cruise = QA_SPEED_CRUISE;
        if (telemetryGetRoverId() == 11 && (millis() - mission_start_ms < 3000)) {
            current_cruise = QA_SPEED_CRUISE * 0.6f;
        }

        if (checkAbort(udpCmd)) {
            setLedColor(255, 0, 0); 
            break; 
        }
        if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) { stopMotors(); delay(50); continue; }
        
        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;
        
        // --- SISTEMA ANTI-SECUESTRO (Soltar el cubo incorrecto) ---
        bool wrong_cube_grabbed = false;
        for (int i=0; i<3; i++) {
            if (!snap.cubes[i].detected || snap.cubes[i].color == target_color) continue;
            
            float dx_cube = snap.cubes[i].x - x;
            float dy_cube = snap.cubes[i].y - y;
            float dist_c = sqrt(dx_cube*dx_cube + dy_cube*dy_cube);
            
            float angle_to_c = atan2(-dy_cube, dx_cube) * 180.0f / M_PI;
            if (angle_to_c < 0) angle_to_c += 360.0f;
            
            float diff_c = angle_to_c - h;
            while (diff_c <= -180.0f) diff_c += 360.0f;
            while (diff_c > 180.0f) diff_c -= 360.0f;
            
            // Si hay un cubo de OTRO color atrapado a menos de 4.5 celdas y justo enfrente (< 35 grados)
            if (dist_c < 4.5f && fabs(diff_c) < 35.0f) {
                wrong_cube_grabbed = true;
                break;
            }
        }
        
        if (wrong_cube_grabbed && (state == DRIVE_PRE || state == SENSOR_APPROACH)) {
            Serial.println("[HUNT] ERROR: Cubo incorrecto secuestrado en pinzas! Abortando acercamiento...");
            stopMotors();
            delay(200);
            
            // Retroceder para escupir el cubo (y salir de su influencia magnetica)
            setMotors(-0.45f, -0.45f);
            delay(1200);
            
            stopMotors();
            state = TURN_PRE; // Volver a empezar (el APF calculara una ruta curva para rodear el obstaculo)
            continue;
        }
        // --------------------------------------------------------
        
        // ACTUALIZACION DINAMICA DE RUTA (Si el cubo se mueve o la camara ajusta la coordenada)
        if (state < TURN_PUSH && snap.cubes[target_color].detected) {
            cube_x = snap.cubes[target_color].x;
            cube_y = snap.cubes[target_color].y;
            
            float depot_x = snap.depots[target_color].x;
            float depot_y = snap.depots[target_color].y;
            
            float dx = cube_x - depot_x;
            float dy = cube_y - depot_y;
            float len = sqrt(dx*dx + dy*dy);
            if (len == 0.0f) len = 1.0f;
            
            pre_x = cube_x + (dx/len) * current_pre_dist;
            pre_y = cube_y + (dy/len) * current_pre_dist;
            pre_x = constrain(pre_x, 3.0f, snap.grid_cols - 3.0f);
            pre_y = constrain(pre_y, 3.0f, snap.grid_rows - 3.0f);

            push_target_x = depot_x + (dx/len) * 4.0f;
            push_target_y = depot_y + (dy/len) * 4.0f;
        }
        
        // ---------------------------------------------------------
        // SISTEMA DE INTERSECCION Y TRAFICO
        // ---------------------------------------------------------
        if (!ignore_traffic && snap.peer_rover.detected && state != DRIVE_PUSH && state != TURN_PUSH) {
            float peer_dist = sqrt(pow(x - snap.peer_rover.x, 2) + pow(y - snap.peer_rover.y, 2));
            
            if (peer_dist < 20.0f && telemetryGetRoverId() == 11) {
                if (!is_yielding) {
                    is_yielding = true;
                    yield_start_ms = millis();
                }
                
                if (millis() - yield_start_ms > 4000) {
                    Serial.println("[TRAFICO] Rover 10 inactivo. Ignorando regla de trafico para rodearlo.");
                    ignore_traffic = true;
                } else {
                    stopMotors();
                    Serial.println("[TRAFICO] Cediendo el paso al Rover 10...");
                    delay(100);
                    continue; 
                }
            } else {
                is_yielding = false;
            }
        }
        
        // El viejo sistema de Bug-0 avoidance fue reemplazado por APF dinámico
        
        debug_path_p1_x = x; debug_path_p1_y = y;
        
        float target_x = pre_x;
        float target_y = pre_y;
        
        if (state == TURN_AVOID || state == DRIVE_AVOID) {
            target_x = avoid_x;
            target_y = avoid_y;
        } else if (state == SENSOR_APPROACH || state == VERIFY_TELEMETRY_CUBE || state == TURN_APPROACH) {
            target_x = cube_x;
            target_y = cube_y;
        } else if (state == TURN_PUSH || state == DRIVE_PUSH) {
            target_x = push_target_x;
            target_y = push_target_y;
        }

        debug_path_p2_x = target_x; debug_path_p2_y = target_y;
        debug_path_p3_x = (state == TURN_PUSH || state == DRIVE_PUSH) ? push_target_x : depot_x; 
        debug_path_p3_y = (state == TURN_PUSH || state == DRIVE_PUSH) ? push_target_y : depot_y;

        float dist = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));
        float dx_target = target_x - x;
        float dy_target = target_y - y;
        float dx_rover = cos(h * M_PI / 180.0f);
        float dy_rover = -sin(h * M_PI / 180.0f); 
        float dot_prod = dx_target * dx_rover + dy_target * dy_rover;
        bool waypoint_reached = (dist < CORNER_ARRIVE_TOL_CELLS) || (dist < 4.0f && dot_prod < 0);
        bool strict_waypoint_reached = (dist < 2.0f) || (dist < 4.0f && dot_prod < 0);
        
        // Calculo de Angulo Dinamico usando Evasion APF (Artificial Potential Fields)
        float target_h = applyPotentialFields(x, y, target_x, target_y, snap, target_color);
        
        if (state != last_state) {
            if (state == TURN_PRE || state == TURN_APPROACH || state == TURN_PUSH || state == TURN_AVOID) {
                syncImuToCamera(h);
            }
        }
        last_state = state;
        
        float diff = target_h - getImuHeading();
        while (diff <= -180.0f) diff += 360.0f;
        while (diff > 180.0f) diff -= 360.0f;

        if (state == TURN_PRE || state == TURN_PUSH || state == TURN_AVOID || state == TURN_APPROACH) {
            static float turn_integral = 0;
            if (fabs(diff) < 20.0f) { // Llegada angular exitosa
                stopMotors();
                turn_integral = 0;
                if (state == TURN_PRE) state = DRIVE_PRE;
                else if (state == TURN_PUSH) state = DRIVE_PUSH;
                else if (state == TURN_AVOID) state = DRIVE_AVOID;
                else if (state == TURN_APPROACH) state = SENSOR_APPROACH;
                delay(150);
            } else {
                turn_integral += diff * 0.02f; // Anti-atasco
                turn_integral = constrain(turn_integral, -0.3f, 0.3f);
                float turn_speed = diff * 0.008f + turn_integral * 0.05f;
                if (turn_speed > 0 && turn_speed < 0.15f) turn_speed = 0.15f;
                if (turn_speed < 0 && turn_speed > -0.15f) turn_speed = -0.15f;
                turn_speed = constrain(turn_speed, -QA_SPEED_PIVOT, QA_SPEED_PIVOT);
                setMotors(-turn_speed, turn_speed);
            }
        } else if (state == DRIVE_AVOID) {
            if (waypoint_reached) {
                stopMotors();
                Serial.println("[HUNT] Waypoint de desvio alcanzado. Retomando ruta...");
                state = TURN_PRE;
                delay(300);
            } else {
                float correction = 0.0f;
                // MODO CARRETERA (Trazos largos): Menos correcciones para evitar zig-zag
                float dyn_deadband = (dist > 12.0f) ? 8.0f : ANGLE_DEADBAND_DEG;
                float dyn_kp = (dist > 12.0f) ? (KP_STEERING * 0.4f) : KP_STEERING;
                
                if (fabs(diff) > dyn_deadband) correction = diff * dyn_kp;
                correction = constrain(correction, -0.12f, 0.12f);
                
                // Rampa de frenado dinamica
                float speed_factor = 1.0f;
                if (dist < 4.0f) speed_factor = dist / 4.0f;
                speed_factor = max(0.5f, speed_factor); // Minimo 50% para no atascarse
                
                setMotors((current_cruise * speed_factor) - correction, (current_cruise * speed_factor) + correction);
            }
        } else if (state == DRIVE_PRE) {
            if (strict_waypoint_reached) { // Llegada estricta al punto matemático pre_x, pre_y
                stopMotors();
                Serial.println("[HUNT] Pre-approach alcanzado. Verificando telemetria...");
                state = VERIFY_TELEMETRY_CUBE;
                wait_start_ms = millis();
                delay(300);
            } else {
                float correction = 0.0f;
                // MODO CARRETERA (Trazos largos): Menos correcciones para evitar zig-zag
                float dyn_deadband = (dist > 12.0f) ? 8.0f : ANGLE_DEADBAND_DEG;
                float dyn_kp = (dist > 12.0f) ? (KP_STEERING * 0.4f) : KP_STEERING;
                
                if (fabs(diff) > dyn_deadband) correction = diff * dyn_kp;
                correction = constrain(correction, -0.12f, 0.12f);
                
                // Rampa de frenado dinamica
                float speed_factor = 1.0f;
                if (dist < 4.0f) speed_factor = dist / 4.0f;
                speed_factor = max(0.5f, speed_factor); // Minimo 50% para no atascarse
                
                setMotors((current_cruise * speed_factor) - correction, (current_cruise * speed_factor) + correction);
            }
        } else if (state == VERIFY_TELEMETRY_CUBE) {
            stopMotors();
            float c_x = -1, c_y = -1;
            for(int i=0; i<3; i++) {
                if (snap.cubes[i].color == target_color && snap.cubes[i].detected) {
                    c_x = snap.cubes[i].x; c_y = snap.cubes[i].y;
                }
            }
            if (c_x >= 0) {
                if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < (current_pre_dist + 3.0f)) {
                    Serial.println("[HUNT] Cubo cerca, pivotando hacia el cubo antes de avanzar...");
                    state = TURN_APPROACH;
                }
            }
            if (millis() - wait_start_ms > 3000) {
                Serial.println("[HUNT] La camara no confirma el cubo. Abortando.");
                setLedColor(255, 0, 0);
                break;
            }
        } else if (state == SENSOR_APPROACH) {
            float sonar_cm = readUltrasonicCm();
            
            if (sonar_cm > 0.0f && sonar_cm < 8.0f) {
                stopMotors();
                Serial.println("[HUNT] Contacto fisico detectado. Verificando color del cubo...");
                delay(500); 
                
                bool color_ok = verifyCubeColor(target_color);
                
                // Si el sensor de color falla o no esta conectado, usar telemetria fresca como fallback estricto
                if (!color_ok) {
                    TelemetrySnapshot fresh_snap;
                    if (telemetryGetSnapshot(fresh_snap) && fresh_snap.cubes[target_color].detected) {
                        float cx = fresh_snap.cubes[target_color].x;
                        float cy = fresh_snap.cubes[target_color].y;
                        float grip_dist = sqrt(pow(cx - x, 2) + pow(cy - y, 2));
                        if (grip_dist < 10.0f) {
                            Serial.println("[HUNT] Sensor de color fallo, pero Camara confirma que el cubo correcto esta en las pinzas.");
                            color_ok = true;
                        }
                    }
                }
                
                if (color_ok) {
                    Serial.println("[HUNT] CONFIRMADO: Cubo capturado.");
                    setLedColor(0, 255, 128);
                    state = TURN_PUSH;
                } else {
                    Serial.println("[HUNT] ERROR: Color incorrecto o no hay cubo. Iniciando BACKUP_RETRY.");
                    setLedColor(255, 128, 0);
                    state = BACKUP_RETRY;
                    wait_start_ms = millis();
                }
            } else {
                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                setMotors(0.18f - correction, 0.18f + correction);
            }
        } else if (state == DRIVE_PUSH) {
            if (waypoint_reached) {
                stopMotors();
                Serial.println("[HUNT] Deposito alcanzado. Retrocediendo...");
                state = BACKUP_AWAY;
                wait_start_ms = millis();
            } else {
                float correction = 0.0f;
                // MODO CARRETERA (Trazos largos): Menos correcciones para evitar zig-zag
                float dyn_deadband = (dist > 12.0f) ? 8.0f : ANGLE_DEADBAND_DEG;
                float dyn_kp = (dist > 12.0f) ? (KP_STEERING * 0.4f) : KP_STEERING;
                
                if (fabs(diff) > dyn_deadband) correction = diff * dyn_kp;
                correction = constrain(correction, -0.12f, 0.12f);
                
                // Rampa de frenado dinamica
                float speed_factor = 1.0f;
                if (dist < 4.0f) speed_factor = dist / 4.0f;
                speed_factor = max(0.5f, speed_factor); // Minimo 50% para no atascarse
                
                setMotors((current_cruise * speed_factor) - correction, (current_cruise * speed_factor) + correction);
            }
        } else if (state == BACKUP_RETRY) {
            if (millis() - wait_start_ms > 1000) {
                stopMotors();
                state = TURN_APPROACH;
            } else {
                setMotors(-0.25f, -0.25f);
            }
        } else if (state == BACKUP_AWAY) {
            if (millis() - wait_start_ms > 1000) {
                stopMotors();
                while(Serial.available()) Serial.read();
                udpCmd.flush();
                state = VERIFY_DROP;
                wait_start_ms = millis();
            } else {
                setMotors(-0.25f, -0.25f);
            }
        } else if (state == VERIFY_DROP) {
            stopMotors();
            float c_x = -1, c_y = -1;
            for(int i=0; i<3; i++) {
                if (snap.cubes[i].color == target_color && snap.cubes[i].detected) {
                    c_x = snap.cubes[i].x; c_y = snap.cubes[i].y;
                }
            }
            if (c_x >= 0) {
                if (sqrt(pow(c_x - depot_x, 2) + pow(c_y - depot_y, 2)) < 6.0f) {
                    setLedColor(0, 255, 0);
                    success = true;
                    break;
                }
            }
            if (millis() - wait_start_ms > 2000) {
                setLedColor(255, 0, 0);
                break;
            }
        }
        delay(20);
    }
    
    active_hunt_color = COLOR_UNKNOWN;
    debug_path_p1_x = -1; debug_path_p1_y = -1;
    debug_path_p2_x = -1; debug_path_p2_y = -1;
    debug_path_p3_x = -1; debug_path_p3_y = -1;
    stopMotors();
    return success;
}

void hunt_multiple_cubes(int count) {
    Serial.printf("\n[MULTI] Iniciando mision multi-cubo (Objetivo: %d cubos)\n", count);
    extern WiFiUDP udpCmd;
    bool completed_cubes[3] = {false, false, false};
    int success_count = 0;
    
    while (success_count < count) {
        if (checkAbort(udpCmd)) {
            Serial.println("[MULTI] Mision multi-cubo abortada por usuario.");
            break;
        }
        
        TelemetrySnapshot snap;
        if (!telemetryGetSnapshot(snap)) {
            delay(100);
            continue;
        }
        
        CubeColor target = get_closest_pending_cube(snap, completed_cubes);
        if (target == COLOR_UNKNOWN) {
            Serial.println("[MULTI] No hay mas cubos pendientes disponibles.");
            break;
        }
        
        bool ok = hunt_cube(target);
        if (ok) {
            Serial.printf("[MULTI] Cubo %d completado.\n", target);
            completed_cubes[target] = true;
            success_count++;
            
            if (success_count < count) {
                Serial.println("[MULTI] Preparando busqueda del siguiente cubo...");
                delay(1500);
            }
        } else {
            Serial.println("[MULTI] Fallo al cazar el cubo o fue abortado. Cancelando resto de la mision.");
            break;
        }
    }
    
    if (success_count == count) {
        Serial.println("[MULTI] ¡MISION MULTI-CUBO COMPLETADA CON EXITO!");
    }
}

void hunt_reto_mission() {
    simulate_routes_mode = false;
    Serial.println("\n[RETO] INICIANDO MODO RETO GLOBAL");
    extern WiFiUDP udpCmd;
    
    int my_id = telemetryGetRoverId();
    int peer_id = telemetryGetPeerRoverId(); // Asumiendo que 10 y 11
    if (peer_id == -1) {
        peer_id = (my_id == 10) ? 11 : 10;
    }
    
    // Algoritmo Greedy de asignación determinista
    TelemetrySnapshot snap;
    while(true) {
        if (!telemetryGetSnapshot(snap)) {
            delay(100);
            if (checkAbort(udpCmd)) return;
            continue;
        }
        break; // Telemetría obtenida
    }
    
    bool cube_assigned[3] = {false, false, false};
    int assigned_to[3] = {-1, -1, -1};
    
    float rx_10, ry_10, rx_11, ry_11;
    if (my_id == 10) {
        rx_10 = snap.my_rover.x; ry_10 = snap.my_rover.y;
        rx_11 = snap.peer_rover.detected ? snap.peer_rover.x : 999.0f;
        ry_11 = snap.peer_rover.detected ? snap.peer_rover.y : 999.0f;
    } else {
        rx_11 = snap.my_rover.x; ry_11 = snap.my_rover.y;
        rx_10 = snap.peer_rover.detected ? snap.peer_rover.x : 999.0f;
        ry_10 = snap.peer_rover.detected ? snap.peer_rover.y : 999.0f;
    }
    
    // --- NUEVA LÓGICA DE ASIGNACIÓN (Minimización del Costo Total) ---
    float cost_matrix[2][3];
    for (int c=0; c<3; c++) {
        if (snap.cubes[c].detected) {
            cost_matrix[0][c] = sqrt(pow(snap.cubes[c].x - rx_10, 2) + pow(snap.cubes[c].y - ry_10, 2));
            cost_matrix[1][c] = sqrt(pow(snap.cubes[c].x - rx_11, 2) + pow(snap.cubes[c].y - ry_11, 2));
        } else {
            cost_matrix[0][c] = 9999.0f; cost_matrix[1][c] = 9999.0f;
        }
    }

    int best_r10_c = -1; int best_r11_c = -1;
    float min_total = 99999.0f;

    for (int c10=0; c10<3; c10++) {
        for (int c11=0; c11<3; c11++) {
            if (c10 == c11) continue; 
            float total = cost_matrix[0][c10] + cost_matrix[1][c11];
            if (total < min_total) {
                min_total = total;
                best_r10_c = c10;
                best_r11_c = c11;
            }
        }
    }
    
    // Asignar los dos primeros cubos
    if (best_r10_c != -1) assigned_to[best_r10_c] = 10;
    if (best_r11_c != -1) assigned_to[best_r11_c] = 11;
    
    // Asignar el 3er cubo sobrante al rover mas cercano a el
    for (int c=0; c<3; c++) {
        if (c != best_r10_c && c != best_r11_c) {
            if (cost_matrix[0][c] < cost_matrix[1][c]) {
                assigned_to[c] = 10;
            } else {
                assigned_to[c] = 11;
            }
        }
    }
    
    for (int c=0; c<3; c++) {
        if (assigned_to[c] != -1) {
            Serial.printf("[RETO] Cubo %d asignado al Rover %d\n", c, assigned_to[c]);
        }
    }
    
    // Desincronizacion Espacial (Temporal reemplazada por límite de velocidad)
    if (my_id == 10) {
        setPreApproachDistance(15.0f); 
        Serial.println("[RETO] Estrategia: Alineacion Temprana (Rover 10)");
    } else {
        setPreApproachDistance(8.0f);  
        Serial.println("[RETO] Estrategia: Alineacion Tardia (Rover 11). Limite vel. inicial 60%.");
    }
    
    // --- LOGICA DE FASE 1: SECUENCIACION ---
    int mis_cubos = 0;
    int sus_cubos = 0;
    for (int c = 0; c < 3; c++) {
        if (assigned_to[c] == my_id) mis_cubos++;
        if (assigned_to[c] == peer_id) sus_cubos++;
    }

    if (mis_cubos == 1 && sus_cubos == 2) {
        Serial.println("[RETO] FASE 1: Me toco 1 solo cubo. Esperare pacientemente a que el companero termine los suyos...");
        while (true) {
            extern WiFiUDP udpCmd;
            if (checkAbort(udpCmd)) return;
            
            TelemetrySnapshot snap_wait;
            if (telemetryGetSnapshot(snap_wait)) {
                // Verificar visualmente si los 2 cubos del companero ya llegaron al deposito
                int cubos_en_deposito = 0;
                for (int c = 0; c < 3; c++) {
                    if (assigned_to[c] == peer_id) {
                        float cx = snap_wait.cubes[c].x; float cy = snap_wait.cubes[c].y;
                        float dx = snap_wait.depots[c].x; float dy = snap_wait.depots[c].y;
                        if (sqrt(pow(cx - dx, 2) + pow(cy - dy, 2)) < 8.0f) {
                            cubos_en_deposito++;
                        }
                    }
                }
                if (cubos_en_deposito == 2) {
                    Serial.println("[RETO] El companero completo su mision. iEs mi turno!");
                    break; // Se levanta el bloqueo y arranca
                }
            }
            delay(500);
        }
    }
    
    // Ejecutar MIS tareas
    for (int c = 0; c < 3; c++) {
        if (assigned_to[c] == my_id) {
            Serial.printf("[RETO] Cazando cubo %d asignado a mi...\n", c);
            bool ok = hunt_cube((CubeColor)c);
            if (!ok) {
                Serial.println("[RETO] Tarea fallida o abortada.");
                return;
            }
            delay(1500); // Pausa entre cubos para no chocar
        }
    }
    
    Serial.println("[RETO] ¡Todas mis tareas del modo reto completadas!");
}
