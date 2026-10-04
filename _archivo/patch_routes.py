import re
import os

def patch_file(filepath):
    with open(filepath, "r") as f:
        content = f.read()

    # 1. Add globals
    globals_code = """
// =============================================================================
// NAVEGACIÓN — Implementación
// =============================================================================

bool simulate_routes_mode = false;
float sim_r10_p1_x = -1, sim_r10_p1_y = -1;
float sim_r10_p2_x = -1, sim_r10_p2_y = -1;
float sim_r10_p3_x = -1, sim_r10_p3_y = -1;

float sim_r11_p1_x = -1, sim_r11_p1_y = -1;
float sim_r11_p2_x = -1, sim_r11_p2_y = -1;
float sim_r11_p3_x = -1, sim_r11_p3_y = -1;

"""
    content = re.sub(r'// =============================================================================\n// NAVEGACIÓN — Implementación\n// =============================================================================\n', globals_code, content)

    # 2. Modify printMap
    old_is_path_block = r'''            // Trazar línea de ruta
            bool is_path = false;
            float cx = x \* scale;
            float cy = y \* scale;
            if \(isOnLineSegment\(cx, cy, debug_path_p1_x, debug_path_p1_y, debug_path_p2_x, debug_path_p2_y\) \|\|
                isOnLineSegment\(cx, cy, debug_path_p2_x, debug_path_p2_y, debug_path_p3_x, debug_path_p3_y\)\) \{
                if \(c == '\.' \|\| c == '#'\) \{
                    c = '\*';
                    is_path = true;
                \}
            \}'''

    new_is_path_block = """            // Trazar línea de ruta
            bool is_path = false;
            bool is_path_10 = false;
            bool is_path_11 = false;
            float cx = x * scale;
            float cy = y * scale;
            
            if (simulate_routes_mode) {
                // Rover 10
                if (isOnLineSegment(cx, cy, sim_r10_p1_x, sim_r10_p1_y, sim_r10_p2_x, sim_r10_p2_y) ||
                    isOnLineSegment(cx, cy, sim_r10_p2_x, sim_r10_p2_y, sim_r10_p3_x, sim_r10_p3_y)) {
                    if (c == '.' || c == '#') {
                        c = '*';
                        if (round(sim_r10_p2_x / scale) == x && round(sim_r10_p2_y / scale) == y) c = '+';
                        is_path_10 = true;
                    }
                }
                // Rover 11
                if (isOnLineSegment(cx, cy, sim_r11_p1_x, sim_r11_p1_y, sim_r11_p2_x, sim_r11_p2_y) ||
                    isOnLineSegment(cx, cy, sim_r11_p2_x, sim_r11_p2_y, sim_r11_p3_x, sim_r11_p3_y)) {
                    if (c == '.' || c == '#') {
                        c = '*';
                        if (round(sim_r11_p2_x / scale) == x && round(sim_r11_p2_y / scale) == y) c = '+';
                        is_path_11 = true;
                    }
                }
            } else {
                if (isOnLineSegment(cx, cy, debug_path_p1_x, debug_path_p1_y, debug_path_p2_x, debug_path_p2_y) ||
                    isOnLineSegment(cx, cy, debug_path_p2_x, debug_path_p2_y, debug_path_p3_x, debug_path_p3_y)) {
                    if (c == '.' || c == '#') {
                        c = '*';
                        is_path = true;
                    }
                }
            }"""
    
    content = re.sub(old_is_path_block, new_is_path_block, content)

    # 3. Modify colors
    old_color_block = r'''            // Colorear e imprimir
            const char\* ansi = "";
            const char\* reset = "\\033\[0m";
            if \(c == 'R' \|\| c == 'r' \|\| \(is_path && active_hunt_color == COLOR_RED\)\) ansi = "\\033\[31m";
            else if \(c == 'G' \|\| c == 'g' \|\| \(is_path && active_hunt_color == COLOR_GREEN\)\) ansi = "\\033\[32m";
            else if \(c == 'B' \|\| c == 'b' \|\| \(is_path && active_hunt_color == COLOR_BLUE\)\) ansi = "\\033\[34m";'''
            
    new_color_block = """            // Colorear e imprimir
            const char* ansi = "";
            const char* reset = "\\033[0m";
            if (simulate_routes_mode && is_path_10) ansi = "\\033[33m"; // Amarillo
            else if (simulate_routes_mode && is_path_11) ansi = "\\033[35m"; // Morado
            else if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = "\\033[31m";
            else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "\\033[32m";
            else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "\\033[34m";"""

    content = re.sub(old_color_block, new_color_block, content)
    
    # 4. Append simulateRoutesTask at the bottom
    new_func = """
void simulateRoutesTask() {
    simulate_routes_mode = true;
    Serial.println("\\n[SIMULATION] MODO IMPRIMIR RUTAS ACTIVADO");
    
    TelemetrySnapshot snap;
    if (!telemetryGetSnapshot(snap)) {
        Serial.println("[SIMULATION] No hay telemetria.");
        return;
    }
    
    float rx_10 = 999.0f, ry_10 = 999.0f;
    float rx_11 = 999.0f, ry_11 = 999.0f;
    
    int my_id = telemetryGetRoverId();
    if (my_id == 10) {
        if (snap.my_rover.detected) { rx_10 = snap.my_rover.x; ry_10 = snap.my_rover.y; }
        if (snap.peer_rover.detected) { rx_11 = snap.peer_rover.x; ry_11 = snap.peer_rover.y; }
    } else {
        if (snap.peer_rover.detected) { rx_10 = snap.peer_rover.x; ry_10 = snap.peer_rover.y; }
        if (snap.my_rover.detected) { rx_11 = snap.my_rover.x; ry_11 = snap.my_rover.y; }
    }
    
    float cost_matrix[2][3];
    for (int c=0; c<3; c++) {
        if (snap.cubes[c].detected) {
            cost_matrix[0][c] = sqrt(pow(snap.cubes[c].x - rx_10, 2) + pow(snap.cubes[c].y - ry_10, 2));
            cost_matrix[1][c] = sqrt(pow(snap.cubes[c].x - rx_11, 2) + pow(snap.cubes[c].y - ry_11, 2));
        } else {
            cost_matrix[0][c] = 9999.0f; cost_matrix[1][c] = 9999.0f;
        }
    }

    int best_r10_c = -1; int best_r11_c = -1;
    float min_total = 99999.0f;
    for (int c10=0; c10<3; c10++) {
        for (int c11=0; c11<3; c11++) {
            if (c10 == c11) continue; 
            float total = cost_matrix[0][c10] + cost_matrix[1][c11];
            if (total < min_total) {
                min_total = total;
                best_r10_c = c10;
                best_r11_c = c11;
            }
        }
    }
    
    if (best_r10_c != -1 && snap.cubes[best_r10_c].detected) {
        sim_r10_p1_x = rx_10; sim_r10_p1_y = ry_10;
        CubeColor col = snap.cubes[best_r10_c].color;
        Vector2D c_pos(snap.cubes[best_r10_c].x, snap.cubes[best_r10_c].y);
        int depot_idx = (col == COLOR_RED) ? 0 : (col == COLOR_GREEN ? 1 : 2);
        Vector2D d_pos(snap.depots[depot_idx].x, snap.depots[depot_idx].y);
        Vector2D pre_app = calculatePreApproach(c_pos, d_pos, getPreApproachDistance());
        sim_r10_p2_x = pre_app.x; sim_r10_p2_y = pre_app.y;
        sim_r10_p3_x = d_pos.x; sim_r10_p3_y = d_pos.y;
    } else { sim_r10_p1_x = -1; }

    if (best_r11_c != -1 && snap.cubes[best_r11_c].detected) {
        sim_r11_p1_x = rx_11; sim_r11_p1_y = ry_11;
        CubeColor col = snap.cubes[best_r11_c].color;
        Vector2D c_pos(snap.cubes[best_r11_c].x, snap.cubes[best_r11_c].y);
        int depot_idx = (col == COLOR_RED) ? 0 : (col == COLOR_GREEN ? 1 : 2);
        Vector2D d_pos(snap.depots[depot_idx].x, snap.depots[depot_idx].y);
        Vector2D pre_app = calculatePreApproach(c_pos, d_pos, getPreApproachDistance());
        sim_r11_p2_x = pre_app.x; sim_r11_p2_y = pre_app.y;
        sim_r11_p3_x = d_pos.x; sim_r11_p3_y = d_pos.y;
    } else { sim_r11_p1_x = -1; }
    
    // Forzar envio inmediato
    printMap(snap);
}
"""
    if "simulateRoutesTask" not in content:
        content += new_func

    # Deactivate simulate_routes_mode on normal movement modes
    content = content.replace("void hunt_reto_mission() {", "void hunt_reto_mission() {\n    simulate_routes_mode = false;")
    content = content.replace("void hunt_cube(CubeColor target_color) {", "void hunt_cube(CubeColor target_color) {\n    simulate_routes_mode = false;")
    content = content.replace("void patrol_border() {", "void patrol_border() {\n    simulate_routes_mode = false;")
    
    with open(filepath, "w") as f:
        f.write(content)

patch_file("rover_qa/rover_1/src/navigation.cpp")
patch_file("rover_qa/rover_2/src/navigation.cpp")
