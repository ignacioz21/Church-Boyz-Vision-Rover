import sys, os

# 1. Update mando.py
with open('mando.py', 'r') as f: mando = f.read()
if "Alterar distancia de rotación" not in mando:
    mando = mando.replace('print("  r : STOP DE EMERGENCIA (Detener motores)")', 'print("  r : STOP DE EMERGENCIA (Detener motores)")\nprint("  d : Alterar distancia de rotacion al cubo")')
    loop_patch = """    if cmd.startswith('d') or cmd.startswith('D'):
        try:
            val = input("Ingresa la nueva distancia en bloques (ej. 8.5): ")
            payload = f"D:{float(val)}"
            sock.sendto(payload.encode(), (UDP_IP, UDP_PORT))
            print(f"-> {payload} enviado.")
        except ValueError:
            print("Valor invalido.")
        continue
        
    cmd_char = cmd.strip()[0]"""
    mando = mando.replace("cmd_char = cmd.strip()[0]", loop_patch)
    with open('mando.py', 'w') as f: f.write(mando)

# 2. Update C++ code for both rovers
rovers = ['rover_qa/rover_1', 'rover_qa/rover_2']
for r in rovers:
    if not os.path.exists(r): continue
    
    # navigation.h
    with open(f"{r}/include/navigation.h", 'r') as f: nav_h = f.read()
    if "void setPreApproachDistance" not in nav_h:
        nav_h = nav_h.replace("void stopNavigation();", "void stopNavigation();\nvoid setPreApproachDistance(float blocks);\nfloat getPreApproachDistance();")
        with open(f"{r}/include/navigation.h", 'w') as f: f.write(nav_h)
        
    # navigation.cpp
    with open(f"{r}/src/navigation.cpp", 'r') as f: nav_cpp = f.read()
    if "static float current_pre_dist" not in nav_cpp:
        nav_cpp = nav_cpp.replace('#include "navigation.h"', '#include "navigation.h"\n\nstatic float current_pre_dist = 10.0f;\nvoid setPreApproachDistance(float blocks) { current_pre_dist = blocks; }\nfloat getPreApproachDistance() { return current_pre_dist; }\n')
        
        # Replace hardcoded 10.0f, 10.5f, 13.0f with dynamic logic
        nav_cpp = nav_cpp.replace("float pre_x = cube_x + (dx/len) * 10.0f;", "float pre_x = cube_x + (dx/len) * current_pre_dist;")
        nav_cpp = nav_cpp.replace("float pre_y = cube_y + (dy/len) * 10.0f;", "float pre_y = cube_y + (dy/len) * current_pre_dist;")
        # No need to change DRIVE_PRE condition because we changed it to `dist < 2.0f` earlier!
        # But wait, what about VERIFY_TELEMETRY_CUBE trigger `sqrt(...) < 13.0f`?
        nav_cpp = nav_cpp.replace("if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < 13.0f) {", "if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < (current_pre_dist + 3.0f)) {")
        
        with open(f"{r}/src/navigation.cpp", 'w') as f: f.write(nav_cpp)
        
    # .ino file
    ino_file = f"{r}/{os.path.basename(r)}.ino"
    with open(ino_file, 'r') as f: ino_cpp = f.read()
    if "cmd == 'D'" not in ino_cpp:
        ino_patch = """        } else if (cmd == 'D') {
            packetBuffer[len] = '\\0';
            float new_dist = atof(&packetBuffer[2]);
            if (new_dist >= 2.0f && new_dist <= 20.0f) {
                setPreApproachDistance(new_dist);
                Serial.printf("\\n>>> [DISTANCIA] Rotacion pre-approach ajustada a %.1f bloques\\n", new_dist);
            }
        } else if (cmd == 'c' || cmd == 'C') {"""
        ino_cpp = ino_cpp.replace("} else if (cmd == 'c' || cmd == 'C') {", ino_patch)
        with open(ino_file, 'w') as f: f.write(ino_cpp)

print("Patch dynamic distance applied.")
