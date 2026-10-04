import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Fix push coordinates for 4-block claws
    old_push = """            float push_x = d_pos.x - (dx/len) * 2.0f;
            float push_y = d_pos.y - (dy/len) * 2.0f;"""
            
    new_push = """            // Pinzas largas: detenerse 4 bloques ANTES del centro del deposito para no chocar con la pared
            float push_x = d_pos.x + (dx/len) * 4.0f;
            float push_y = d_pos.y + (dy/len) * 4.0f;"""
            
    content = content.replace(old_push, new_push)

    # 2. Add short backup when DRIVE_PUSH finishes
    old_done = """            if (state == DRIVE_PUSH) {
                if (waypoint_reached) {
                    stopMotors();
                    Serial.println("[HUNT] Cubo depositado exitosamente.");
                    active_hunt_color = COLOR_UNKNOWN;
                    return true;
                }
            }"""
            
    new_done = """            if (state == DRIVE_PUSH) {
                if (waypoint_reached) {
                    stopMotors();
                    Serial.println("[HUNT] Cubo depositado. Retrocediendo para soltar pinzas...");
                    setMotors(-0.4f, -0.4f);
                    delay(800); // Retrocede aprox 4-6 bloques
                    stopMotors();
                    active_hunt_color = COLOR_UNKNOWN;
                    return true;
                }
            }"""
            
    content = content.replace(old_done, new_done)

    # 3. Add go_home() function
    go_home_func = """
void go_home() {
    Serial.println("[HOME] Regresando a la base (Aproximacion a 5.0, 21.0)...");
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
        
        float target_x = 5.0f; // Zona de inicio tipica
        float target_y = 21.5f;
        
        float dx = target_x - x;
        float dy = target_y - y;
        float dist = sqrt(dx*dx + dy*dy);
        
        if (dist < 5.0f) {
            stopMotors();
            Serial.println("[HOME] Base alcanzada. Estacionado.");
            return;
        }
        
        float target_h = atan2(-dy, dx) * 180.0f / M_PI;
        if (target_h < 0) target_h += 360.0f;
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
"""
    # Insert go_home before hunt_cube
    if "void go_home()" not in content:
        content = content.replace("bool hunt_cube(CubeColor target_color)", go_home_func + "\nbool hunt_cube(CubeColor target_color)")

    with open(filepath, 'w') as f:
        f.write(content)

patch("rover_qa/rover_1/src/navigation.cpp")
patch("rover_qa/rover_2/src/navigation.cpp")
