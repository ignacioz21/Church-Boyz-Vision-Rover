import sys

with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'r') as f:
    content = f.read()
    
start_idx = content.find('void hunt_cube(CubeColor target_color) {')
if start_idx == -1:
    print("Error: hunt_cube not found")
    sys.exit(1)
    
before_hunt = content[:start_idx]

new_code = """bool calculateAvoidanceWaypoint(float rx, float ry, float tx, float ty, const TelemetrySnapshot &snap, CubeColor target_color, float &out_wx, float &out_wy) {
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

void hunt_cube(CubeColor target_color) {
    Serial.printf("\\n[HUNT] Iniciando mision: Buscar y depositar cubo %d...\\n", target_color);
    extern WiFiUDP udpCmd;
    active_hunt_color = target_color;
    
    TelemetrySnapshot snap;
    if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) {
        Serial.println("[HUNT] Sin telemetria inicial. Abortando.");
        active_hunt_color = COLOR_UNKNOWN;
        return;
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
        return;
    }

    float dx = cube_x - depot_x;
    float dy = cube_y - depot_y;
    float len = sqrt(dx*dx + dy*dy);
    if (len == 0.0f) len = 1.0f;
    
    // ZONA SEGURA: Reducida a 3 celdas (antes 6)
    float pre_x = cube_x + (dx/len) * 3.0f;
    float pre_y = cube_y + (dy/len) * 3.0f;

    // TARGET EMPUJE: offset de 4 celdas para asegurar entrada en la zona
    float push_target_x = depot_x + (dx/len) * 4.0f;
    float push_target_y = depot_y + (dy/len) * 4.0f;

    enum HuntState { TURN_PRE, DRIVE_PRE, TURN_AVOID, DRIVE_AVOID, VERIFY_TELEMETRY_CUBE, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH, BACKUP_AWAY, VERIFY_DROP };
    HuntState state = TURN_PRE;
    unsigned long wait_start_ms = 0;
    float avoid_x = 0, avoid_y = 0;
    
    setLedColor(255, 128, 0); // Naranja: buscando

    while (true) {
        if (checkAbort(udpCmd)) {
            setLedColor(255, 0, 0); 
            break; 
        }
        if (!telemetryGetSnapshot(snap) || !snap.my_rover.detected) { stopMotors(); delay(50); continue; }
        
        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;
        
        // EVASION DE OBSTACULOS
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
        } else if (state == SENSOR_APPROACH || state == VERIFY_TELEMETRY_CUBE) {
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

        if (state == TURN_PRE || state == TURN_PUSH || state == TURN_AVOID) {
            if (fabs(diff) < 5.0f) {
                stopMotors();
                if (state == TURN_PRE) state = DRIVE_PRE;
                else if (state == TURN_PUSH) state = DRIVE_PUSH;
                else if (state == TURN_AVOID) state = DRIVE_AVOID;
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
                // ANTI-ZIGZAG
                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                setMotors(QA_SPEED_CRUISE - correction, QA_SPEED_CRUISE + correction);
            }
        } else if (state == DRIVE_PRE) {
            if (dist < CORNER_ARRIVE_TOL_CELLS) {
                stopMotors();
                Serial.println("[HUNT] Pre-approach alcanzado. Verificando telemetria...");
                state = VERIFY_TELEMETRY_CUBE;
                wait_start_ms = millis();
                delay(300);
            } else {
                // ANTI-ZIGZAG
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
                if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < 6.0f) {
                    Serial.println("[HUNT] Cubo cerca, iniciando SONAR approach.");
                    state = SENSOR_APPROACH;
                }
            }
            if (millis() - wait_start_ms > 3000) {
                Serial.println("[HUNT] La camara no confirma el cubo. Abortando.");
                setLedColor(255, 0, 0);
                break;
            }
        } else if (state == SENSOR_APPROACH) {
            float sonar_cm = readUltrasonicCm();
            if (sonar_cm > 0.0f && sonar_cm < 6.0f) {
                stopMotors();
                Serial.println("[HUNT] Fisicamente interceptado. Check de color...");
                delay(500);
                if (verifyCubeColor(target_color)) {
                    Serial.println("[HUNT] Color OK. STATUS SOSTENIDO.");
                    setLedColor(0, 255, 128); // Cian: Sostenido
                    state = TURN_PUSH;
                } else {
                    setLedColor(255, 0, 0);
                    break;
                }
            } else {
                // ANTI-ZIGZAG
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
                // ANTI-ZIGZAG
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
                    setLedColor(0, 255, 0); // Verde Éxito
                    break;
                }
            }
            if (millis() - wait_start_ms > 2000) {
                setLedColor(255, 0, 0); // Rojo error
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
}
"""
with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'w') as f:
    f.write(before_hunt + new_code)
