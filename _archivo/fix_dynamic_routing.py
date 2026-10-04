import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()

    # Find where x, y, h are updated
    update_anchor = "        float h = snap.my_rover.heading;"
    
    dynamic_update = """        float h = snap.my_rover.heading;
        
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
        
    text = text.replace(update_anchor, dynamic_update)

    with open(filepath, 'w') as f:
        f.write(text)

fix_file('rover_qa/rover_1/src/navigation.cpp')
fix_file('rover_qa/rover_2/src/navigation.cpp')
