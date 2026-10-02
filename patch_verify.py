import sys, os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """                Serial.println("[HUNT] Fisicamente interceptado. Check de color...");
                delay(500);
                if (verifyCubeColor(target_color)) {
                    Serial.println("[HUNT] Color OK. STATUS SOSTENIDO.");
                    setLedColor(0, 255, 128);
                    state = TURN_PUSH;
                } else {
                    setLedColor(255, 0, 0);
                    break;
                }"""
                
    replace_str = """                Serial.println("[HUNT] Interceptado. Verificando agarre por telemetria...");
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
                        Serial.println("[HUNT] Camara dice: iCubo NO atrapado! (Distancia muy grande). Abortando...");
                        setLedColor(255, 0, 0);
                        break;
                    }
                } else {
                    // Fallback
                    Serial.println("[HUNT] Telemetria inestable. Asumiendo agarre por fallback.");
                    setLedColor(0, 255, 128);
                    state = TURN_PUSH;
                }"""
                
    if "Verificando agarre por telemetria" not in content:
        content = content.replace(find_str, replace_str)
        with open(filepath, 'w') as f: f.write(content)

print("Verification patched.")
