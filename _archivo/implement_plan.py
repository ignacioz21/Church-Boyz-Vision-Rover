import os

def patch_file(filepath, old_str, new_str):
    with open(filepath, 'r') as f:
        content = f.read()
    content = content.replace(old_str, new_str)
    with open(filepath, 'w') as f:
        f.write(content)

# 1. central_brain.py
patch_file("central_brain.py",
"""        self.last_targets = {10: -1, 11: -1}""",
"""        self.last_targets = {10: -1, 11: -1}
        self.home_positions = {10: None, 11: None}
        self.has_worked = {10: False, 11: False}""")

patch_file("central_brain.py",
"""                if line.startswith("B:"):
                    self.mode = line.split("B:")[1].strip()""",
"""                if line.startswith("B:"):
                    new_mode = line.split("B:")[1].strip()
                    if new_mode in ["EXEC_10", "EXEC_11", "EXEC_ALL", "AUTO"] and self.mode == "IDLE":
                        self.home_positions = {10: None, 11: None}
                        self.has_worked = {10: False, 11: False}
                    self.mode = new_mode""")

patch_file("central_brain.py",
"""            if target_idx == -1:
                # No hay cubos asignados (termino o no hay)
                if self.states[rid] != "PARKED":
                    self.udp_sock.sendto(f"{rid}:H".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.states[rid] = "PARKED"
                continue
                
            if target_idx < len(cubes):""",
"""            if target_idx == -1:
                # No hay cubos asignados. Solo va a casa si TRABAJO.
                if self.has_worked[rid] and self.states[rid] != "PARKED":
                    hx, hy = self.home_positions[rid] if self.home_positions[rid] else (5.0, 21.5)
                    self.udp_sock.sendto(f"{rid}:H,{hx:.2f},{hy:.2f}".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.states[rid] = "PARKED"
                continue
                
            if target_idx < len(cubes):
                # Va a cazar. Guardamos su posicion inicial si no la tenemos.
                if self.home_positions[rid] is None:
                    rover_data = next((r for r in msg.get("rovers", []) if r.get("id") == rid), None)
                    if rover_data:
                        self.home_positions[rid] = (rover_data.get("col", 5.0), rover_data.get("row", 21.5))
                self.has_worked[rid] = True""")

# 2. navigation.h
for rid in [1, 2]:
    patch_file(f"rover_qa/rover_{rid}/include/navigation.h", 
               "void go_home();", 
               "void go_home(float target_x, float target_y);")

# 3. navigation.cpp
old_go_home = """void go_home() {
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
        float target_y = 21.5f;"""
        
new_go_home = """void go_home(float target_x, float target_y) {
    Serial.printf("[HOME] Regresando a punto de salida exacto (%.2f, %.2f)...\\n", target_x, target_y);
    unsigned long start_time = millis();
    while (millis() - start_time < 10000) { // Timeout de 10 seg
        TelemetrySnapshot snap;
        if (!telemetryGetSnapshot(snap)) {
            delay(50);
            continue;
        }
        
        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;"""

for rid in [1, 2]:
    patch_file(f"rover_qa/rover_{rid}/src/navigation.cpp", old_go_home, new_go_home)

# 4. rover_1.ino and rover_2.ino
old_ino = """        } else if (cmd == 'h' || cmd == 'H') {
            Serial.println("\\n[MENU] >>> REGRESANDO A CASA...");
            go_home();
        }"""
        
new_ino = """        } else if (cmd == 'h' || cmd == 'H') {
            float hx = 5.0f, hy = 21.5f;
            int comma1 = str_cmd.indexOf(',');
            if (comma1 != -1) {
                int comma2 = str_cmd.indexOf(',', comma1 + 1);
                hx = str_cmd.substring(comma1 + 1, comma2).toFloat();
                hy = str_cmd.substring(comma2 + 1).toFloat();
            }
            Serial.printf("\\n[MENU] >>> REGRESANDO A CASA EXACTA: %.2f, %.2f\\n", hx, hy);
            go_home(hx, hy);
        }"""

for rid in [1, 2]:
    patch_file(f"rover_qa/rover_{rid}/rover_{rid}.ino", old_ino, new_ino)

print("Patch applied successfully.")
