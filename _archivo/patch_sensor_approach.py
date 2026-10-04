import os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """        } else if (state == SENSOR_APPROACH) {
            float sonar_cm = readUltrasonicCm();
            if (sonar_cm > 0.0f && sonar_cm < 12.0f) {"""
            
    replace_str = """        } else if (state == SENSOR_APPROACH) {
            float sonar_cm = readUltrasonicCm();
            float dist_to_cube_map = sqrt(pow(cube_x - x, 2) + pow(cube_y - y, 2));
            
            // Atrapa el cubo si el sonar lo ve, O si en el mapa ya estamos a menos de 7 bloques (14 cm)
            // Esto compensa si las pinzas son muy largas y no dejan que el cubo toque el sonar
            if ((sonar_cm > 0.0f && sonar_cm < 12.0f) || dist_to_cube_map < 7.0f) {"""
            
    content = content.replace(find_str, replace_str)
    
    with open(filepath, 'w') as f: f.write(content)

print("Sensor approach patched to use map.")
