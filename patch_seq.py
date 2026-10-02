import sys, os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """    // Ejecutar MIS tareas"""
    
    inject_str = """    // --- LOGICA DE FASE 1: SECUENCIACION ---
    int mis_cubos = 0;
    int sus_cubos = 0;
    for (int c = 0; c < 3; c++) {
        if (assigned_to[c] == my_id) mis_cubos++;
        if (assigned_to[c] == peer_id) sus_cubos++;
    }

    if (mis_cubos == 1 && sus_cubos == 2) {
        Serial.println("[RETO] FASE 1: Me toco 1 solo cubo. Esperare pacientemente a que el companero termine los suyos...");
        while (true) {
            extern WiFiUDP udpCmd;
            if (checkAbort(udpCmd)) return;
            
            TelemetrySnapshot snap_wait;
            if (telemetryGetSnapshot(snap_wait)) {
                // Verificar visualmente si los 2 cubos del companero ya llegaron al deposito
                int cubos_en_deposito = 0;
                for (int c = 0; c < 3; c++) {
                    if (assigned_to[c] == peer_id) {
                        float cx = snap_wait.cubes[c].x; float cy = snap_wait.cubes[c].y;
                        float dx = snap_wait.depots[c].x; float dy = snap_wait.depots[c].y;
                        if (sqrt(pow(cx - dx, 2) + pow(cy - dy, 2)) < 8.0f) {
                            cubos_en_deposito++;
                        }
                    }
                }
                if (cubos_en_deposito == 2) {
                    Serial.println("[RETO] El companero completo su mision. iEs mi turno!");
                    break; // Se levanta el bloqueo y arranca
                }
            }
            delay(500);
        }
    }
    
    // Ejecutar MIS tareas"""
    
    if "LOGICA DE FASE 1: SECUENCIACION" not in content:
        content = content.replace(find_str, inject_str)
        with open(filepath, 'w') as f: f.write(content)

print("Sequencing patched.")
