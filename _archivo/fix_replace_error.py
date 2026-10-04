import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()

    bad_block = """        float h = snap.my_rover.heading;
        
        // ACTUALIZACION DINAMICA DE RUTA (Si el cubo se mueve o la camara ajusta la coordenada)
        if (state < TURN_PUSH && snap.cubes[target_color].detected) {
            cube_x = snap.cubes[target_color].x;
            cube_y = snap.cubes[target_color].y;
            
            float depot_x = snap.depots[target_color].x;
            float depot_y = snap.depots[target_color].y;
            
            float dx = cube_x - depot_x;
            float dy = cube_y - depot_y;
            float len = sqrt(dx*dx + dy*dy);
            if (len == 0.0f) len = 1.0f;
            
            pre_x = cube_x + (dx/len) * current_pre_dist;
            pre_y = cube_y + (dy/len) * current_pre_dist;
            pre_x = constrain(pre_x, 3.0f, snap.grid_cols - 3.0f);
            pre_y = constrain(pre_y, 3.0f, snap.grid_rows - 3.0f);

            push_target_x = depot_x + (dx/len) * 4.0f;
            push_target_y = depot_y + (dy/len) * 4.0f;
        }"""
        
    good_block = """        float h = snap.my_rover.heading;"""

    # First revert ALL instances
    text = text.replace(bad_block, good_block)

    # Now carefully apply it ONLY inside hunt_cube
    start_idx = text.find('bool hunt_cube(CubeColor target_color)')
    end_idx = text.find('void hunt_multiple_cubes', start_idx)

    hunt_cube_str = text[start_idx:end_idx]
    hunt_cube_str = hunt_cube_str.replace(good_block, bad_block, 1) # Only replace the first occurrence in hunt_cube

    text = text[:start_idx] + hunt_cube_str + text[end_idx:]

    with open(filepath, 'w') as f:
        f.write(text)

fix_file('rover_qa/rover_1/src/navigation.cpp')
fix_file('rover_qa/rover_2/src/navigation.cpp')
