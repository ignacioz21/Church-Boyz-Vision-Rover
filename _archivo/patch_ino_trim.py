import sys, os

files = [
    'rover_qa/rover_1/rover_1.ino',
    'rover_qa/rover_2/rover_2.ino'
]

for filepath in files:
    if not os.path.exists(filepath): continue
    
    with open(filepath, 'r') as f: content = f.read()
    
    # Locate the stop motors command block to inject before it
    find_str = """        } else if (cmd == 'r' || cmd == 'R') {"""
    
    inject_str = """        } else if (cmd == '[') {
            float tl = getMotorTrimL() - 0.05f;
            if (tl < 0.3f) tl = 1.0f;
            setMotorTrim(tl, getMotorTrimR());
            Serial.printf("\\n>>> [TRIM] Motor IZQUIERDO bajado a %.2f (%.0f%%)\\n", tl, tl*100);
        } else if (cmd == ']') {
            float tr = getMotorTrimR() - 0.05f;
            if (tr < 0.3f) tr = 1.0f;
            setMotorTrim(getMotorTrimL(), tr);
            Serial.printf("\\n>>> [TRIM] Motor DERECHO bajado a %.2f (%.0f%%)\\n", tr, tr*100);
        } else if (cmd == '{' || cmd == '}') {
            setMotorTrim(1.0f, 1.0f);
            Serial.println("\\n>>> [TRIM] Ambos motores restaurados al 100%\\n");
        } else if (cmd == 'r' || cmd == 'R') {"""
        
    content = content.replace(find_str, inject_str)
    
    with open(filepath, 'w') as f: f.write(content)

print("INO trim commands injected.")
