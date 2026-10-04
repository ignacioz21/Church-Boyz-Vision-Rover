import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Add timeout for motors
    if "unsigned long last_motor_cmd_ms = 0;" not in content:
        content = content.replace("void setup() {", "unsigned long last_motor_cmd_ms = 0;\n\nvoid setup() {")

    # Add the M command parser
    cmd_m = """        } else if (cmd == 'M') {
            int comma_idx = str_cmd.indexOf(',', 2);
            if (comma_idx != -1) {
                float ml = str_cmd.substring(2, comma_idx).toFloat();
                float mr = str_cmd.substring(comma_idx + 1).toFloat();
                setMotors(ml, mr);
                last_motor_cmd_ms = millis();
            }
        }"""
        
    # Replace the old M command (which was printMap)
    old_m = """        } else if (cmd == 'm' || cmd == 'M') {
            TelemetrySnapshot snap;
            if (telemetryGetSnapshot(snap)) {
                // printMap(snap); // Dibujo delegado a Python
            } else {
                Serial.println("[MAP] Sin datos de telemetria.");
            }
        }"""
        
    if old_m in content:
        content = content.replace(old_m, cmd_m)
        
    # Add watchdog in loop
    watchdog = """    if (millis() - last_motor_cmd_ms > 500 && last_motor_cmd_ms != 0) {
        stopMotors();
        last_motor_cmd_ms = 0;
        Serial.println("[WATCHDOG] Conexion perdida. Motores detenidos.");
    }

    // =========================================================================
    // 5. DIAGNÓSTICO DE MOTORES (Mando Manual 'f', 'i', 'l')"""
    
    content = content.replace("    // =========================================================================\n    // 5. DIAGNÓSTICO DE MOTORES (Mando Manual 'f', 'i', 'l')", watchdog)

    with open(filepath, 'w') as f:
        f.write(content)

patch("rover_qa/rover_1/rover_1.ino")
patch("rover_qa/rover_2/rover_2.ino")
