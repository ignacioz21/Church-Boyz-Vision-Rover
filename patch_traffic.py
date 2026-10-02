import sys, os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;"""
        
    replace_str = """        float x = snap.my_rover.x;
        float y = snap.my_rover.y;
        float h = snap.my_rover.heading;
        
        // ---------------------------------------------------------
        // SISTEMA DE INTERSECCION (CEDA EL PASO EN EL CENTRO)
        // ---------------------------------------------------------
        float mid_x = snap.grid_cols / 2.0f;
        
        // ¿Estoy yo en la zona central? (Franja de +- 10 bloques)
        bool im_in_intersection = abs(x - mid_x) < 10.0f;
        
        if (im_in_intersection && snap.peer_rover.detected) {
            bool peer_in_intersection = abs(snap.peer_rover.x - mid_x) < 12.0f;
            
            // Regla de prioridad: Rover 10 tiene prioridad, Rover 11 Cede el Paso
            if (peer_in_intersection && telemetryGetRoverId() == 11) {
                stopMotors();
                Serial.println("[TRAFICO] Alto. Cediendo el paso al Rover 10 en la interseccion central...");
                delay(100);
                continue; // Espera hasta que el Rover 10 despeje la zona
            }
        }"""
        
    if "SISTEMA DE INTERSECCION" not in content:
        content = content.replace(find_str, replace_str)
        with open(filepath, 'w') as f: f.write(content)

print("Traffic intersection logic applied.")
