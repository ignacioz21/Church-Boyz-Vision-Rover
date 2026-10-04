import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()

    old_sensor = """            if (sonar_cm > 0.0f && sonar_cm < 8.0f) {
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
            }"""

    new_sensor = """            if (sonar_cm > 0.0f && sonar_cm < 8.0f) {
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
            }"""

    text = text.replace(old_sensor, new_sensor)

    with open(filepath, 'w') as f:
        f.write(text)

fix_file('rover_qa/rover_1/src/navigation.cpp')
fix_file('rover_qa/rover_2/src/navigation.cpp')
