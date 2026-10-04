import os

files = [
    'rover_qa/rover_1/rover_1.ino',
    'rover_qa/rover_2/rover_2.ino'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    # 1. Fix str_cmd parsing to allow longer commands
    content = content.replace("if (str_cmd.length() == 1) {", "if (str_cmd.length() >= 1) {")
    
    # 2. Replace the broken D command block
    bad_block = """                } else if (cmd == 'D') {
            packetBuffer[len] = '\\0';
            float new_dist = atof(&packetBuffer[2]);
            if (new_dist >= 2.0f && new_dist <= 20.0f) {
                setPreApproachDistance(new_dist);
                Serial.printf("\\n>>> [DISTANCIA] Rotacion pre-approach ajustada a %.1f bloques\\n", new_dist);
            }"""
            
    good_block = """        } else if (cmd == 'D') {
            float new_dist = str_cmd.substring(2).toFloat();
            if (new_dist >= 2.0f && new_dist <= 20.0f) {
                setPreApproachDistance(new_dist);
                Serial.printf("\\n>>> [DISTANCIA] Rotacion pre-approach ajustada a %.1f bloques\\n", new_dist);
            }"""
            
    content = content.replace(bad_block, good_block)
    
    with open(filepath, 'w') as f: f.write(content)

print("Fixed scope in INO.")
