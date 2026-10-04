import os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    # Clamp pre_x and pre_y
    find_str1 = """    float pre_x = cube_x + (dx/len) * current_pre_dist;
    float pre_y = cube_y + (dy/len) * current_pre_dist;"""
    
    replace_str1 = """    float pre_x = cube_x + (dx/len) * current_pre_dist;
    float pre_y = cube_y + (dy/len) * current_pre_dist;
    pre_x = constrain(pre_x, 3.0f, snap.grid_cols - 3.0f);
    pre_y = constrain(pre_y, 3.0f, snap.grid_rows - 3.0f);"""
    
    content = content.replace(find_str1, replace_str1)
    
    # Reduce 30.0f to 15.0f
    find_str2 = """setPreApproachDistance(30.0f);"""
    replace_str2 = """setPreApproachDistance(15.0f);"""
    content = content.replace(find_str2, replace_str2)
    
    with open(filepath, 'w') as f: f.write(content)

print("Crazy 10 fixed.")
