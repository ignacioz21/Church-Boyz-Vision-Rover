import sys

with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'r') as f:
    content = f.read()

# 1. Update calculateAvoidanceWaypoint to include peer_rover
old_avoid = """    for (int i = 0; i < 3; i++) {
        if (snap.cubes[i].color == target_color || !snap.cubes[i].detected) continue;
        
        float ox = snap.cubes[i].x;
        float oy = snap.cubes[i].y;"""
        
new_avoid = """    // Chequear al OTRO rover como obstaculo principal (mayor margen de seguridad)
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
        float oy = snap.cubes[i].y;"""

content = content.replace(old_avoid, new_avoid)

# 2. Add hunt_reto_mission function
reto_func = """
void hunt_reto_mission() {
    Serial.println("\\n[RETO] INICIANDO MODO RETO GLOBAL");
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
            Serial.printf("[RETO] Cubo %d asignado al Rover %d (dist: %.1f)\\n", best_c, best_rover, min_dist);
        }
    }
    
    // Ejecutar MIS tareas
    for (int c = 0; c < 3; c++) {
        if (assigned_to[c] == my_id) {
            Serial.printf("[RETO] Cazando cubo %d asignado a mi...\\n", c);
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
"""

content = content + reto_func

with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'w') as f:
    f.write(content)

