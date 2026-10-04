import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_sensor = """        } else if (state == SENSOR_APPROACH) {
            float c_x = -1, c_y = -1;
            for(int i=0; i<3; i++) {
                if (snap.cubes[i].color == target_color && snap.cubes[i].detected) {
                    c_x = snap.cubes[i].x; c_y = snap.cubes[i].y;
                }
            }
            if (c_x < 0) {
                stopMotors();
                state = TURN_PRE; // Volver a empezar
                continue;
            }
            
            float dist_to_cube = sqrt(pow(c_x - x, 2) + pow(c_y - y, 2));"""
            
    new_sensor = """        } else if (state == SENSOR_APPROACH) {
            float c_x = cube_x; 
            float c_y = cube_y;
            for(int i=0; i<3; i++) {
                if (snap.cubes[i].color == target_color && snap.cubes[i].detected) {
                    c_x = snap.cubes[i].x; c_y = snap.cubes[i].y;
                    cube_x = c_x; cube_y = c_y; // Actualizar memoria
                }
            }
            
            float dist_to_cube = sqrt(pow(c_x - x, 2) + pow(c_y - y, 2));"""
            
    content = content.replace(old_sensor, new_sensor)

    with open(filepath, 'w') as f:
        f.write(content)

for rid in [1, 2]:
    patch(f"rover_qa/rover_{rid}/src/navigation.cpp")
print("Sensor approach flicker patched")
