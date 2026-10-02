import sys, os

files = [
    ('rover_qa/rover_1/include/hardware.h', 'rover_qa/rover_1/src/hardware.cpp', 'rover_qa/rover_1/rover_1.ino'),
    ('rover_qa/rover_2/include/hardware.h', 'rover_qa/rover_2/src/hardware.cpp', 'rover_qa/rover_2/rover_2.ino')
]

for hw_h, hw_cpp, ino in files:
    if not os.path.exists(hw_h): continue

    # 1. Patch hardware.h
    with open(hw_h, 'r') as f: h_content = f.read()
    h_patch = """void setMotorInversions(bool invert_l, bool invert_r);
bool getMotorInvertL();
bool getMotorInvertR();
void setMotorTrim(float trim_l, float trim_r);
float getMotorTrimL();
float getMotorTrimR();"""
    h_content = h_content.replace("void setMotorInversions(bool invert_l, bool invert_r);\nbool getMotorInvertL();\nbool getMotorInvertR();", h_patch)
    with open(hw_h, 'w') as f: f.write(h_content)

    # 2. Patch hardware.cpp
    with open(hw_cpp, 'r') as f: cpp_content = f.read()
    
    cpp_patch_1 = """static bool current_invert_l = INVERT_MOTOR_L;
static bool current_invert_r = INVERT_MOTOR_R;
static float current_trim_l = MOTOR_TRIM_L;
static float current_trim_r = MOTOR_TRIM_R;

void setMotorTrim(float trim_l, float trim_r) {
    current_trim_l = trim_l;
    current_trim_r = trim_r;
}
float getMotorTrimL() { return current_trim_l; }
float getMotorTrimR() { return current_trim_r; }"""

    cpp_content = cpp_content.replace("static bool current_invert_l = INVERT_MOTOR_L;\nstatic bool current_invert_r = INVERT_MOTOR_R;", cpp_patch_1)
    
    # In setMotors, apply trim
    cpp_patch_2 = """    // Aplicar Trim
    left *= current_trim_l;
    right *= current_trim_r;

    // Aplicar inversión"""
    cpp_content = cpp_content.replace("    // Aplicar inversión", cpp_patch_2)
    with open(hw_cpp, 'w') as f: f.write(cpp_content)

    # 3. Patch INO
    with open(ino, 'r') as f: ino_content = f.read()
    
    ino_patch_1 = """    Serial.println("  [c] Alternar ID del Rover (10 <-> 11)");
    Serial.println("  [[] Bajar trim motor IZQUIERDO (-5%)");
    Serial.println("  []] Bajar trim motor DERECHO (-5%)");
    Serial.println("  [{] o [}] Restaurar trims al 100%");"""
    ino_content = ino_content.replace('    Serial.println("  [c] Alternar ID del Rover (10 <-> 11)");', ino_patch_1)
    
    ino_patch_2 = """        } else if (cmd == 'c' || cmd == 'C') {
            int current = telemetryGetRoverId();
            int nuevo = (current == 10) ? 11 : 10;
            telemetrySetRoverId(nuevo, (nuevo == 10) ? 11 : 10);
            Serial.printf("\\n>>> [ID] Rover cambiado a ID %d\\n", nuevo);
        } else if (cmd == '[') {
            float tl = getMotorTrimL() - 0.05f;
            if (tl < 0.3f) tl = 1.0f;
            setMotorTrim(tl, getMotorTrimR());
            Serial.printf("\\n>>> [TRIM] Motor IZQ ajustado a %.2f\\n", tl);
        } else if (cmd == ']') {
            float tr = getMotorTrimR() - 0.05f;
            if (tr < 0.3f) tr = 1.0f;
            setMotorTrim(getMotorTrimL(), tr);
            Serial.printf("\\n>>> [TRIM] Motor DER ajustado a %.2f\\n", tr);
        } else if (cmd == '{' || cmd == '}') {
            setMotorTrim(1.0f, 1.0f);
            Serial.println("\\n>>> [TRIM] Motores restaurados a 100% (1.00)\\n");
        } else if (cmd == 'p' || cmd == 'P') {"""
        
    ino_content = ino_content.replace("""        } else if (cmd == 'c' || cmd == 'C') {
            int current = telemetryGetRoverId();
            int nuevo = (current == 10) ? 11 : 10;
            telemetrySetRoverId(nuevo, (nuevo == 10) ? 11 : 10);
            Serial.printf("\\n>>> [ID] Rover cambiado a ID %d\\n", nuevo);
        } else if (cmd == 'p' || cmd == 'P') {""", ino_patch_2)
        
    with open(ino, 'w') as f: f.write(ino_content)

print("Trim patches applied.")
