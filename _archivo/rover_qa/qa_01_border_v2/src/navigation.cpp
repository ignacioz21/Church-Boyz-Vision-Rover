#include "../include/navigation.h"
#include "../include/config.h"
#include "../include/telemetry.h"
#include "../include/hardware.h"
#include <math.h>
#include <WiFiUdp.h>

static WiFiUDP udpMap;
static bool udpInitialized = false;

// =============================================================================
// NAVEGACIÓN — Implementación
// =============================================================================

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
    return sqrt(pow(px - proj_x, 2) + pow(py - proj_y, 2)) <= 1.0f; // Tolerancia 1 celda
}

void printMap(const TelemetrySnapshot &snap) {
    if (!udpInitialized) {
        udpMap.begin(8888);
        udpInitialized = true;
    }

    // Escala: 2 celdas = 1 caracter
    const float scale = 2.0f;
    int map_w = round(snap.grid_cols / scale);
    int map_h = round(snap.grid_rows / scale);

    if (map_w <= 0 || map_h <= 0 || map_w > 70) {
        return; // Grid invalido
    }

    int margin = round(EDGE_CLEARANCE_CELLS / scale);

    // Iniciar paquete UDP directo a la PC
    udpMap.beginPacket(VISION_HOST, 8888);
    
    udpMap.println("=== MAPA DE CAMPO EN VIVO (Escala 1 char = 2 celdas) ===");
    
    // Borde superior
    udpMap.print("+");
    for (int x = 0; x < map_w; x++) udpMap.print("--");
    udpMap.println("+");

    // Filas
    for (int y = 0; y < map_h; y++) {
        udpMap.print("|");
        for (int x = 0; x < map_w; x++) {
            char c = '.'; // vacio

            bool in_margin = (x < margin || x >= map_w - margin || y < margin || y >= map_h - margin);

            // 1. Zona de holgura
            if (in_margin) {
                c = '#';
            }

            // 2. Depositos
            for(int i = 0; i < 3; i++) {
                if (snap.depots[i].x > 0 && round(snap.depots[i].x / scale) == x && round(snap.depots[i].y / scale) == y) {
                    if (i == 0) c = 'r';
                    else if (i == 1) c = 'g';
                    else if (i == 2) c = 'b';
                }
            }

            // 3. Cubos
            for(int i = 0; i < 3; i++) {
                if (snap.cubes[i].detected && round(snap.cubes[i].x / scale) == x && round(snap.cubes[i].y / scale) == y) {
                    if (snap.cubes[i].color == COLOR_RED) c = 'R';
                    else if (snap.cubes[i].color == COLOR_GREEN) c = 'G';
                    else if (snap.cubes[i].color == COLOR_BLUE) c = 'B';
                }
            }

            // 4. Rover compañero
            if (snap.peer_rover.detected && round(snap.peer_rover.x / scale) == x && round(snap.peer_rover.y / scale) == y) {
                c = 'O';
            }

            // Target WP actual (visual)
            if (debug_target_x >= 0 && round(debug_target_x / scale) == x && round(debug_target_y / scale) == y) {
                c = '*';
            }

            // Trazar línea de ruta
            bool is_path = false;
            float cx = x * scale;
            float cy = y * scale;
            if (isOnLineSegment(cx, cy, debug_path_p1_x, debug_path_p1_y, debug_path_p2_x, debug_path_p2_y) ||
                isOnLineSegment(cx, cy, debug_path_p2_x, debug_path_p2_y, debug_path_p3_x, debug_path_p3_y)) {
                if (c == '.' || c == '#') {
                    c = '*';
                    is_path = true;
                }
            }

            // 5. Mi Rover
            if (snap.my_rover.detected && round(snap.my_rover.x / scale) == x && round(snap.my_rover.y / scale) == y) {
                float h = snap.my_rover.heading;
                if (h >= 315.0f || h < 45.0f) c = '>';
                else if (h >= 45.0f && h < 135.0f) c = '^';
                else if (h >= 135.0f && h < 225.0f) c = '<';
                else c = 'v';
            }

            // Colorear e imprimir
            const char* ansi = "";
            const char* reset = "\033[0m";
            if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = "\033[31m";
            else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "\033[32m";
            else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "\033[34m";
            else if (c == 'O') ansi = "\033[33m"; // Compañero (Amarillo)
            else if (c == '>' || c == '<' || c == '^' || c == 'v') ansi = "\033[1;37m"; // Blanco Brillante para el rover
            
            if (ansi[0] != '\0') {
                udpMap.printf("%s%c%s", ansi, c, reset);
            } else {
                udpMap.print(c);
            }

            if (in_margin && !is_path && c == '#') udpMap.print('#'); 
            else udpMap.print(' ');
        }
        udpMap.println("|");
    }

    // Borde inferior
    udpMap.print("+");
    for (int x = 0; x < map_w; x++) udpMap.print("--");
    udpMap.println("+");

    udpMap.printf("Pos:(%.1f, %.1f) Heading:%.0f deg | Fase:%d\n",
        snap.my_rover.x, snap.my_rover.y, snap.my_rover.heading, snap.phase);
        
    udpMap.endPacket(); // Enviar todo el mapa de una vez
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
        float vec_to_obst_x = closest_obst_x - (rx + min_dist_along * path_nx);
        float vec_to_obst_y = closest_obst_y - (ry + min_dist_along * path_ny);
        
        float side_nx = -path_ny;
        float side_ny = path_nx;
        if (vec_to_obst_x * side_nx + vec_to_obst_y * side_ny > 0) {
            side_nx = path_ny;
            side_ny = -path_nx;
        }
        
        out_wx = rx + min_dist_along * path_nx + side_nx * 5.0f;
        out_wy = ry + min_dist_along * path_ny + side_ny * 5.0f;
        
        out_wx = constrain(out_wx, 2.0f, snap.grid_cols - 2.0f);
        out_wy = constrain(out_wy, 2.0f, snap.grid_rows - 2.0f);
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
    
    float pre_x = cube_x + (dx/len) * 5.0f;
    float pre_y = cube_y + (dy/len) * 5.0f;

    float push_target_x = depot_x + (dx/len) * 4.0f;
    float push_target_y = depot_y + (dy/len) * 4.0f;

    enum HuntState { TURN_PRE, DRIVE_PRE, TURN_AVOID, DRIVE_AVOID, VERIFY_TELEMETRY_CUBE, TURN_APPROACH, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH, BACKUP_AWAY, VERIFY_DROP };
    HuntState state = TURN_PRE;
    unsigned long wait_start_ms = 0;
    float avoid_x = 0, avoid_y = 0;
    
    setLedColor(255, 128, 0); 
    bool success = false;

    while (true) {
        if (checkAbort(udpCmd)) {
            setLedColor(255, 0, 0); 
            break; 
        }
        if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) { stopMotors(); delay(50); continue; }
        
        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;
        
        if (state == TURN_PRE || state == DRIVE_PRE) {
            if (calculateAvoidanceWaypoint(x, y, pre_x, pre_y, snap, target_color, avoid_x, avoid_y)) {
                Serial.println("[HUNT] ¡Obstaculo detectado (otro cubo)! Generando waypoint de desvio...");
                state = TURN_AVOID;
            }
        }
        
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
        float target_h = atan2(-(target_y - y), (target_x - x)) * 180.0f / M_PI;
        if (target_h < 0) target_h += 360.0f;
        
        float diff = target_h - h;
        while (diff <= -180.0f) diff += 360.0f;
        while (diff > 180.0f) diff -= 360.0f;

        if (state == TURN_PRE || state == TURN_PUSH || state == TURN_AVOID || state == TURN_APPROACH) {
            if (fabs(diff) < 5.0f) {
                stopMotors();
                if (state == TURN_PRE) state = DRIVE_PRE;
                else if (state == TURN_PUSH) state = DRIVE_PUSH;
                else if (state == TURN_AVOID) state = DRIVE_AVOID;
                else if (state == TURN_APPROACH) state = SENSOR_APPROACH;
                delay(300);
            } else {
                float turn_speed = diff * 0.008f;
                if (turn_speed > 0 && turn_speed < 0.18f) turn_speed = 0.18f;
                if (turn_speed < 0 && turn_speed > -0.18f) turn_speed = -0.18f;
                turn_speed = constrain(turn_speed, -QA_SPEED_PIVOT, QA_SPEED_PIVOT);
                setMotors(-turn_speed, turn_speed);
            }
        } else if (state == DRIVE_AVOID) {
            if (dist < CORNER_ARRIVE_TOL_CELLS) {
                stopMotors();
                Serial.println("[HUNT] Waypoint de desvio alcanzado. Retomando ruta...");
                state = TURN_PRE;
                delay(300);
            } else {
                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                setMotors(QA_SPEED_CRUISE - correction, QA_SPEED_CRUISE + correction);
            }
        } else if (state == DRIVE_PRE) {
            float dist_to_cube = sqrt(pow(cube_x - x, 2) + pow(cube_y - y, 2));
            if (dist_to_cube <= 5.5f) { // Frena exactamente a 5.5 bloques del cubo
                stopMotors();
                Serial.println("[HUNT] Pre-approach alcanzado. Verificando telemetria...");
                state = VERIFY_TELEMETRY_CUBE;
                wait_start_ms = millis();
                delay(300);
            } else {
                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                setMotors(QA_SPEED_CRUISE - correction, QA_SPEED_CRUISE + correction);
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
                if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < 8.0f) {
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
            if (sonar_cm > 0.0f && sonar_cm < 12.0f) {
                stopMotors();
                Serial.println("[HUNT] Fisicamente interceptado. Check de color...");
                delay(500);
                if (verifyCubeColor(target_color)) {
                    Serial.println("[HUNT] Color OK. STATUS SOSTENIDO.");
                    setLedColor(0, 255, 128);
                    state = TURN_PUSH;
                } else {
                    setLedColor(255, 0, 0);
                    break;
                }
            } else {
                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                float slow = 0.17f;
                setMotors(slow - correction, slow + correction);
            }
        } else if (state == DRIVE_PUSH) {
            if (dist < CORNER_ARRIVE_TOL_CELLS) {
                stopMotors();
                Serial.println("[HUNT] Deposito alcanzado. Retrocediendo...");
                state = BACKUP_AWAY;
                wait_start_ms = millis();
            } else {
                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                setMotors(QA_SPEED_CRUISE - correction, QA_SPEED_CRUISE + correction);
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
    
    float rx_me = snap.my_rover.x;
    float ry_me = snap.my_rover.y;
    float rx_peer = snap.peer_rover.detected ? snap.peer_rover.x : 999.0f; // Si no lo vemos, asumimos lejos
    float ry_peer = snap.peer_rover.detected ? snap.peer_rover.y : 999.0f;
    
    for (int step = 0; step < 3; step++) {
        float min_dist = 99999.0f;
        int best_c = -1;
        int best_rover = -1;
        
        for (int c = 0; c < 3; c++) {
            if (cube_assigned[c] || !snap.cubes[c].detected) continue;
            
            // Distancia a MI
            float d_me = sqrt(pow(snap.cubes[c].x - rx_me, 2) + pow(snap.cubes[c].y - ry_me, 2));
            if (d_me < min_dist) {
                min_dist = d_me;
                best_c = c;
                best_rover = my_id;
            }
            
            // Distancia al PEER
            float d_peer = sqrt(pow(snap.cubes[c].x - rx_peer, 2) + pow(snap.cubes[c].y - ry_peer, 2));
            if (d_peer < min_dist) {
                min_dist = d_peer;
                best_c = c;
                best_rover = peer_id;
            }
        }
        
        if (best_c != -1) {
            cube_assigned[best_c] = true;
            assigned_to[best_c] = best_rover;
            Serial.printf("[RETO] Cubo %d asignado al Rover %d (dist: %.1f)\n", best_c, best_rover, min_dist);
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
