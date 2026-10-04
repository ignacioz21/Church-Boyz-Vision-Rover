import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_verify = """            if (c_x < 0) {
                state = TURN_PRE; // Perdimos el cubo, reintentar
                continue;
            }
            
            float angle_to_cube = atan2(-(c_y - y), c_x - x) * 180.0f / M_PI;"""
            
    new_verify = """            if (c_x < 0) {
                // Filtro contra parpadeos de camara
                if (millis() - wait_start_ms > 1000) {
                    state = TURN_PRE;
                }
                continue;
            }
            
            float angle_to_cube = atan2(-(c_y - y), c_x - x) * 180.0f / M_PI;"""
            
    content = content.replace(old_verify, new_verify)

    with open(filepath, 'w') as f:
        f.write(content)

for rid in [1, 2]:
    patch(f"rover_qa/rover_{rid}/src/navigation.cpp")
print("Flicker protection patched")
