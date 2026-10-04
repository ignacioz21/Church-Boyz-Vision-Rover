def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    watchdog = """
    if (millis() - last_motor_cmd_ms > 500 && last_motor_cmd_ms != 0) {
        stopMotors();
        last_motor_cmd_ms = 0;
        Serial.println("[WATCHDOG] Conexion perdida. Motores detenidos.");
    }
    
    delay(50);"""
    
    content = content.replace("    delay(50);", watchdog)

    with open(filepath, 'w') as f:
        f.write(content)

patch("rover_qa/rover_1/rover_1.ino")
patch("rover_qa/rover_2/rover_2.ino")
