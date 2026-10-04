import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Inject the Artificial Potential Fields function
    apf_func = """
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
        if (dist_to_target > 5.0f) {
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
"""
    if "applyPotentialFields" not in content:
        content = content.replace("bool calculateAvoidanceWaypoint", apf_func + "\nbool calculateAvoidanceWaypoint")
        
    # 2. Disable old static avoidance trigger
    old_trigger = """        if (state == TURN_PRE || state == DRIVE_PRE) {
            if (calculateAvoidanceWaypoint(x, y, pre_x, pre_y, snap, target_color, avoid_x, avoid_y)) {
                Serial.println("[HUNT] ¡Obstaculo detectado (otro cubo)! Generando waypoint de desvio...");
                state = TURN_AVOID;
            }
        }"""
        
    new_trigger = """        // El viejo sistema de Bug-0 avoidance fue reemplazado por APF dinámico"""
    
    content = content.replace(old_trigger, new_trigger)
    
    # 3. Replace basic heading calculation with APF in hunt_cube
    old_h = """        float target_h = atan2(-(target_y - y), (target_x - x)) * 180.0f / M_PI;
        if (target_h < 0) target_h += 360.0f;"""
        
    new_h = """        // Calculo de Angulo Dinamico usando Evasion APF (Artificial Potential Fields)
        float target_h = applyPotentialFields(x, y, target_x, target_y, snap, target_color);"""
        
    content = content.replace(old_h, new_h)
    
    # 4. Replace basic heading calculation with APF in go_home()
    old_home_h = """        float target_h = atan2(-dy, dx) * 180.0f / M_PI;
        if (target_h < 0) target_h += 360.0f;"""
        
    new_home_h = """        float target_h = applyPotentialFields(x, y, target_x, target_y, snap, COLOR_UNKNOWN);"""
    
    content = content.replace(old_home_h, new_home_h)

    with open(filepath, 'w') as f:
        f.write(content)

patch("rover_qa/rover_1/src/navigation.cpp")
patch("rover_qa/rover_2/src/navigation.cpp")
