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

void hunt_cube(CubeColor target_color) {
    Serial.printf("\n[HUNT] Iniciando mision: Buscar y depositar cubo %d...\n", target_color);
    extern WiFiUDP udpCmd;
    active_hunt_color = target_color;
    
    TelemetrySnapshot snap;
    if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) {
        Serial.println("[HUNT] Sin telemetria inicial. Abortando.");
        active_hunt_color = COLOR_UNKNOWN;
        return;
    }

    // Identificar posiciones
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
        return;
    }

    // Calcular el vector desde el depósito hasta el cubo, y extenderlo para el pre-approach
    float dx = cube_x - depot_x;
    float dy = cube_y - depot_y;
    float len = sqrt(dx*dx + dy*dy);
    if (len == 0.0f) len = 1.0f; // Evitar division por cero
    
    // Punto de pre-aproximación: 6 celdas detrás del cubo (en la línea que lo une con el depósito)
    float pre_x = cube_x + (dx/len) * 6.0f;
    float pre_y = cube_y + (dy/len) * 6.0f;

    // Pintar ruta en el dashboard
    debug_path_p1_x = snap.my_rover.x; debug_path_p1_y = snap.my_rover.y;
    debug_path_p2_x = pre_x; debug_path_p2_y = pre_y;
    debug_path_p3_x = depot_x; debug_path_p3_y = depot_y;
    
    enum HuntState { TURN_PRE, DRIVE_PRE, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH };
    HuntState state = TURN_PRE;
    
    while (true) {
        if (checkAbort(udpCmd)) break; // Stop remoto
        if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) { stopMotors(); delay(50); continue; }
        
        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;
        
        debug_path_p1_x = x; debug_path_p1_y = y; // Actualizar origen del dibujo
        
        // Target de navegación según la fase
        float target_x = (state == TURN_PRE || state == DRIVE_PRE || state == SENSOR_APPROACH) ? pre_x : depot_x;
        float target_y = (state == TURN_PRE || state == DRIVE_PRE || state == SENSOR_APPROACH) ? pre_y : depot_y;
        
        if (state == SENSOR_APPROACH) {
            target_x = cube_x;
            target_y = cube_y;
        }

        float dist = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));
        float target_h = atan2(-(target_y - y), (target_x - x)) * 180.0f / M_PI;
        if (target_h < 0) target_h += 360.0f;
        
        float diff = target_h - h;
        while (diff <= -180.0f) diff += 360.0f;
        while (diff > 180.0f) diff -= 360.0f;

        if (state == TURN_PRE || state == TURN_PUSH) {
            if (fabs(diff) < 5.0f) {
                stopMotors();
                state = (state == TURN_PRE) ? DRIVE_PRE : DRIVE_PUSH;
                Serial.println("[HUNT] Enfilado correcto.");
                delay(300);
            } else {
                float turn_speed = diff * 0.008f;
                if (turn_speed > 0 && turn_speed < 0.18f) turn_speed = 0.18f;
                if (turn_speed < 0 && turn_speed > -0.18f) turn_speed = -0.18f;
                turn_speed = constrain(turn_speed, -QA_SPEED_PIVOT, QA_SPEED_PIVOT);
                setMotors(-turn_speed, turn_speed);
            }
        } else if (state == DRIVE_PRE) {
            if (dist < CORNER_ARRIVE_TOL_CELLS) {
                stopMotors();
                Serial.println("[HUNT] Pre-approach alcanzado. Cambiando a APROXIMACION SONAR.");
                state = SENSOR_APPROACH;
                delay(300);
            } else {
                float Kp = 0.008f; float correction = diff * Kp;
                if (correction > 0.15f) correction = 0.15f;
                if (correction < -0.15f) correction = -0.15f;
                setMotors(QA_SPEED_CRUISE - correction, QA_SPEED_CRUISE + correction);
            }
        } else if (state == SENSOR_APPROACH) {
            // Avanzar despacio, pero usando el SONAR para la precisión final
            float sonar_cm = readUltrasonicCm();
            if (sonar_cm > 0.0f && sonar_cm < 6.0f) { // Llegamos fisicamente al cubo!
                stopMotors();
                Serial.println("[HUNT] ¡Cubo interceptado físicamente (Sonar < 6cm)!");
                delay(500); // Estabilizar
                if (verifyCubeColor(target_color)) {
                    Serial.println("[HUNT] Color confirmado localmente. Iniciando empuje (PUSH).");
                    state = TURN_PUSH;
                } else {
                    Serial.println("[HUNT] Color INCORRECTO. Abortando misión.");
                    break;
                }
            } else {
                // Aproximación lenta y controlada
                float Kp = 0.008f; float correction = diff * Kp;
                if (correction > 0.15f) correction = 0.15f;
                if (correction < -0.15f) correction = -0.15f;
                float slow_cruise = 0.17f; // Más lento que Cruise para el docking final
                setMotors(slow_cruise - correction, slow_cruise + correction);
            }
        } else if (state == DRIVE_PUSH) {
            if (dist < CORNER_ARRIVE_TOL_CELLS) {
                stopMotors();
                Serial.println("[HUNT] Depósito alcanzado. ¡Misión de cubo completada!");
                break;
            } else {
                float Kp = 0.008f; float correction = diff * Kp;
                if (correction > 0.15f) correction = 0.15f;
                if (correction < -0.15f) correction = -0.15f;
                setMotors(QA_SPEED_CRUISE - correction, QA_SPEED_CRUISE + correction);
            }
        }
        delay(20);
    }
    
    // Limpiar mapa
    active_hunt_color = COLOR_UNKNOWN;
    debug_path_p1_x = -1; debug_path_p1_y = -1;
    debug_path_p2_x = -1; debug_path_p2_y = -1;
    debug_path_p3_x = -1; debug_path_p3_y = -1;
    stopMotors();
}
