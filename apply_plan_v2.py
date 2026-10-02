import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()

    # 1. Traffic Logic & Variables
    old_init = """    enum HuntState { TURN_PRE, DRIVE_PRE, TURN_AVOID, DRIVE_AVOID, VERIFY_TELEMETRY_CUBE, TURN_APPROACH, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH, BACKUP_AWAY, VERIFY_DROP, BACKUP_RETRY };
    HuntState state = TURN_PRE;
    unsigned long wait_start_ms = 0;
    unsigned long mission_start_ms = millis();
    float avoid_x = 0, avoid_y = 0;"""

    new_init = """    enum HuntState { TURN_PRE, DRIVE_PRE, TURN_AVOID, DRIVE_AVOID, VERIFY_TELEMETRY_CUBE, TURN_APPROACH, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH, BACKUP_AWAY, VERIFY_DROP, BACKUP_RETRY };
    HuntState state = TURN_PRE;
    unsigned long wait_start_ms = 0;
    unsigned long mission_start_ms = millis();
    float avoid_x = 0, avoid_y = 0;
    unsigned long yield_start_ms = 0;
    bool is_yielding = false;
    bool ignore_traffic = false;"""

    text = text.replace(old_init, new_init)

    old_traffic = """        // ---------------------------------------------------------
        // SISTEMA DE INTERSECCION Y TRAFICO
        // ---------------------------------------------------------
        if (snap.peer_rover.detected && state != DRIVE_PUSH && state != TURN_PUSH) {
            float peer_dist = sqrt(pow(x - snap.peer_rover.x, 2) + pow(y - snap.peer_rover.y, 2));
            
            // Si están a menos de 20 celdas (40cm) de distancia, el Rover 11 cede el paso
            if (peer_dist < 20.0f && telemetryGetRoverId() == 11) {
                stopMotors();
                Serial.println("[TRAFICO] Peligro de colision dinamico. Rover 11 cede el paso al Rover 10...");
                delay(100);
                continue; 
            }
        }"""

    new_traffic = """        // ---------------------------------------------------------
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
        }"""

    text = text.replace(old_traffic, new_traffic)

    # 2. SENSOR_APPROACH Anti-Fantasma
    old_sensor = """        } else if (state == SENSOR_APPROACH) {
            float sonar_cm = readUltrasonicCm();
            float dist_to_cube_map = sqrt(pow(cube_x - x, 2) + pow(cube_y - y, 2));
            
            // Atrapa el cubo si el sonar lo ve, O si en el mapa ya estamos a menos de 7 bloques (14 cm)
            // Esto compensa si las pinzas son muy largas y no dejan que el cubo toque el sonar
            if ((sonar_cm > 0.0f && sonar_cm < 12.0f) || dist_to_cube_map < 7.0f) {
                stopMotors();
                Serial.println("[HUNT] Interceptado. Verificando agarre por telemetria...");
                delay(1000); // Dar tiempo a que el cubo se asiente en las pinzas
                
                // Pedimos una foto fresca a la camara
                TelemetrySnapshot fresh_snap;
                if (telemetryGetSnapshot(fresh_snap) && fresh_snap.my_rover.detected && fresh_snap.cubes[target_color].detected) {
                    float fresh_cube_x = fresh_snap.cubes[target_color].x;
                    float fresh_cube_y = fresh_snap.cubes[target_color].y;
                    float fresh_rx = fresh_snap.my_rover.x;
                    float fresh_ry = fresh_snap.my_rover.y;
                    
                    float grip_dist = sqrt(pow(fresh_cube_x - fresh_rx, 2) + pow(fresh_cube_y - fresh_ry, 2));
                    
                    if (grip_dist < 10.0f) { // Tolerancia ampliada a 10 bloques (20cm) para pinzas largas
                        Serial.println("[HUNT] Camara confirma: Cubo atrapado en pinzas.");
                        setLedColor(0, 255, 128);
                        state = TURN_PUSH;
                    } else {
                        Serial.println("[HUNT] Camara dice: iCubo NO atrapado! Reintentando captura...");
                        setLedColor(255, 128, 0);
                        state = BACKUP_RETRY;
                        wait_start_ms = millis();
                    }
                } else {
                    // Fallback
                    Serial.println("[HUNT] Telemetria inestable. Asumiendo agarre por fallback.");
                    setLedColor(0, 255, 128);
                    state = TURN_PUSH;
                }
            } else {
                float correction = 0.0f;
                if (fabs(diff) > ANGLE_DEADBAND_DEG) correction = diff * KP_STEERING;
                correction = constrain(correction, -0.12f, 0.12f);
                float slow = 0.17f;
                setMotors(slow - correction, slow + correction);
            }"""

    new_sensor = """        } else if (state == SENSOR_APPROACH) {
            float sonar_cm = readUltrasonicCm();
            
            if (sonar_cm > 0.0f && sonar_cm < 8.0f) {
                stopMotors();
                Serial.println("[HUNT] Contacto fisico detectado. Verificando color del cubo...");
                delay(500); 
                
                if (verifyCubeColor(target_color)) {
                    Serial.println("[HUNT] CONFIRMADO: Cubo capturado y color correcto.");
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
            }"""

    text = text.replace(old_sensor, new_sensor)

    # 3. Wiggling Fix
    old_turn = """        if (state == TURN_PRE || state == TURN_PUSH || state == TURN_AVOID || state == TURN_APPROACH) {
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
        }"""

    new_turn = """        if (state == TURN_PRE || state == TURN_PUSH || state == TURN_AVOID || state == TURN_APPROACH) {
            if (fabs(diff) < 20.0f) {
                stopMotors();
                if (state == TURN_PRE) state = DRIVE_PRE;
                else if (state == TURN_PUSH) state = DRIVE_PUSH;
                else if (state == TURN_AVOID) state = DRIVE_AVOID;
                else if (state == TURN_APPROACH) state = SENSOR_APPROACH;
                delay(150);
            } else {
                float turn_speed = diff * 0.008f;
                if (turn_speed > 0 && turn_speed < 0.15f) turn_speed = 0.15f;
                if (turn_speed < 0 && turn_speed > -0.15f) turn_speed = -0.15f;
                turn_speed = constrain(turn_speed, -QA_SPEED_PIVOT, QA_SPEED_PIVOT);
                setMotors(-turn_speed, turn_speed);
            }
        }"""

    text = text.replace(old_turn, new_turn)

    with open(filepath, 'w') as f:
        f.write(text)

fix_file('rover_qa/rover_1/src/navigation.cpp')
fix_file('rover_qa/rover_2/src/navigation.cpp')
