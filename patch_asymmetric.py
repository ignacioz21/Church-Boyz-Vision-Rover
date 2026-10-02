import sys, os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """    // Ejecutar MIS tareas"""
    
    inject_str = """    // Desincronizacion Espacial y Temporal para evitar choques de pinzas
    if (my_id == 10) {
        setPreApproachDistance(30.0f); // Se alinea al principio de la ruta (a 60cm del cubo)
        Serial.println("[RETO] Estrategia: Alineacion Temprana (Rover 10)");
    } else {
        setPreApproachDistance(8.0f);  // Se alinea al llegar al objetivo (a 16cm del cubo)
        Serial.println("[RETO] Estrategia: Alineacion Tardia (Rover 11). Esperando 3 segs...");
        delay(3000); // Espera a que el Rover 10 despeje la zona inicial
    }
    
    // Ejecutar MIS tareas"""
    
    if "Desincronizacion Espacial y Temporal" not in content:
        content = content.replace(find_str, inject_str)
        with open(filepath, 'w') as f: f.write(content)

print("Asymmetric logic injected.")
