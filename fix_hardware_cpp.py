import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()

    old_sensor = """    if (!colorSensorConnected) {
        // Si no hay sensor, asumimos que la cámara tiene razón siempre para no bloquear el flujo.
        return true; 
    }"""

    new_sensor = """    if (!colorSensorConnected) {
        // Si no hay sensor, devolver false para obligar al fallback de telemetria
        return false; 
    }"""

    text = text.replace(old_sensor, new_sensor)

    with open(filepath, 'w') as f:
        f.write(text)

fix_file('rover_qa/rover_1/src/hardware.cpp')
fix_file('rover_qa/rover_2/src/hardware.cpp')
