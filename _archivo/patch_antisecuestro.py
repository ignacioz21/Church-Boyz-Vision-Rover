import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Fix APF Exclusion Zone (5.0 -> 2.0)
    old_apf = """        float dist_to_target = sqrt(pow(tx - snap.cubes[i].x, 2) + pow(ty - snap.cubes[i].y, 2));
        if (dist_to_target > 5.0f) {
            add_repulsion(snap.cubes[i].x, snap.cubes[i].y, R_rep);
        }"""
        
    new_apf = """        float dist_to_target = sqrt(pow(tx - snap.cubes[i].x, 2) + pow(ty - snap.cubes[i].y, 2));
        // Reducir tolerancia para que SÍ esquive cubos que estan bloqueando el camino
        if (dist_to_target > 2.0f) {
            add_repulsion(snap.cubes[i].x, snap.cubes[i].y, R_rep);
        }"""
        
    content = content.replace(old_apf, new_apf)

    # 2. Inject Anti-Secuestro logic inside the while(true) loop of hunt_cube
    anchor = """        float h = snap.my_rover.heading;
        
        // ACTUALIZACION DINAMICA DE RUTA (Si el cubo se mueve o la camara ajusta la coordenada)"""
        
    injection = """        float h = snap.my_rover.heading;
        
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
        
        // ACTUALIZACION DINAMICA DE RUTA (Si el cubo se mueve o la camara ajusta la coordenada)"""
        
    if anchor in content:
        content = content.replace(anchor, injection)
    else:
        print("Anchor not found!")

    with open(filepath, 'w') as f:
        f.write(content)

for rid in [1, 2]:
    patch(f"rover_qa/rover_{rid}/src/navigation.cpp")
print("Anti-secuestro patched on both rovers")
