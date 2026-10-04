import sys, os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    if not os.path.exists(filepath): continue
    
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """        } else if (state == DRIVE_PRE) {
            float dist_to_cube = sqrt(pow(cube_x - x, 2) + pow(cube_y - y, 2));
            if (dist_to_cube <= 10.5f) { // Frena exactamente a 5.5 bloques del cubo"""
            
    replace_str = """        } else if (state == DRIVE_PRE) {
            if (dist < 2.0f) { // Llegada estricta al punto matemático pre_x, pre_y"""
            
    content = content.replace(find_str, replace_str)
    
    with open(filepath, 'w') as f: f.write(content)

print("Nav 6 patch applied.")
