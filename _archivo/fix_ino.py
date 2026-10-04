import re

for rover_dir in ["rover_1", "rover_2"]:
    file_path = f"rover_qa/{rover_dir}/{rover_dir}.ino"
    with open(file_path, "r") as f:
        content = f.read()
    
    # Insert the G command handler right after the M command handler
    new_cmd = '''        } else if (cmd == 'M') {
            int comma_idx = str_cmd.indexOf(',', 2);
            if (comma_idx != -1) {
                float ml = str_cmd.substring(2, comma_idx).toFloat();
                float mr = str_cmd.substring(comma_idx + 1).toFloat();
                setMotors(ml, mr);
                last_motor_cmd_ms = millis();
            }
        } else if (cmd == 'G') {
            int comma_idx = str_cmd.indexOf(',', 2);
            if (comma_idx != -1) {
                float target_x = str_cmd.substring(2, comma_idx).toFloat();
                float target_y = str_cmd.substring(comma_idx + 1).toFloat();
                Serial.printf("[GOTO] Navegando a coordenada (%.2f, %.2f)\\n", target_x, target_y);
                go_home(target_x, target_y);
                stopMotors();
            }'''
            
    content = content.replace('''        } else if (cmd == 'M') {
            int comma_idx = str_cmd.indexOf(',', 2);
            if (comma_idx != -1) {
                float ml = str_cmd.substring(2, comma_idx).toFloat();
                float mr = str_cmd.substring(comma_idx + 1).toFloat();
                setMotors(ml, mr);
                last_motor_cmd_ms = millis();
            }''', new_cmd)
            
    with open(file_path, "w") as f:
        f.write(content)
