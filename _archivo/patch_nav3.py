import sys

with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'r') as f:
    lines = f.readlines()

new_lines = []
for line in lines:
    if "float pre_x = cube_x + (dx/len) * 3.0f;" in line:
        new_lines.append(line.replace("3.0f", "5.0f"))
    elif "float pre_y = cube_y + (dy/len) * 3.0f;" in line:
        new_lines.append(line.replace("3.0f", "5.0f"))
    elif "enum HuntState { TURN_PRE, DRIVE_PRE, TURN_AVOID, DRIVE_AVOID, VERIFY_TELEMETRY_CUBE, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH, BACKUP_AWAY, VERIFY_DROP };" in line:
        new_lines.append(line.replace("VERIFY_TELEMETRY_CUBE, SENSOR_APPROACH", "VERIFY_TELEMETRY_CUBE, TURN_APPROACH, SENSOR_APPROACH"))
    elif "if (state == SENSOR_APPROACH || state == VERIFY_TELEMETRY_CUBE)" in line:
        new_lines.append(line.replace("VERIFY_TELEMETRY_CUBE)", "VERIFY_TELEMETRY_CUBE || state == TURN_APPROACH)"))
    elif "if (state == TURN_PRE || state == TURN_PUSH || state == TURN_AVOID)" in line:
        new_lines.append(line.replace("TURN_AVOID)", "TURN_AVOID || state == TURN_APPROACH)"))
    elif "else if (state == TURN_AVOID) state = DRIVE_AVOID;" in line:
        new_lines.append(line + "                else if (state == TURN_APPROACH) state = SENSOR_APPROACH;\n")
    elif "if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < 6.0f) {" in line:
        # Check if we are inside VERIFY_TELEMETRY_CUBE, change 6.0f to 8.0f
        new_lines.append(line.replace("6.0f", "8.0f"))
    elif 'Serial.println("[HUNT] Cubo cerca, iniciando SONAR approach.");' in line:
        new_lines.append('                    Serial.println("[HUNT] Cubo cerca, pivotando hacia el cubo antes de avanzar...");\n')
    elif "state = SENSOR_APPROACH;" in line and "iniciando SONAR approach" in lines[lines.index(line)-1]:
        # Change state transition in VERIFY_TELEMETRY_CUBE
        new_lines.append('                    state = TURN_APPROACH;\n')
    elif "if (sonar_cm > 0.0f && sonar_cm < 6.0f) {" in line:
        new_lines.append(line.replace("6.0f", "12.0f"))
    else:
        new_lines.append(line)

with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'w') as f:
    f.writelines(new_lines)

print("Patch 3 done.")
