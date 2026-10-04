with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'r') as f:
    text = f.read()

find_str = """        } else if (state == DRIVE_PRE) {
            if (dist < CORNER_ARRIVE_TOL_CELLS) {"""
            
replace_str = """        } else if (state == DRIVE_PRE) {
            float dist_to_cube = sqrt(pow(cube_x - x, 2) + pow(cube_y - y, 2));
            if (dist_to_cube <= 5.5f) { // Frena exactamente a 5.5 bloques del cubo"""

text = text.replace(find_str, replace_str)

with open('rover_qa/qa_01_border_v2/src/navigation.cpp', 'w') as f:
    f.write(text)
