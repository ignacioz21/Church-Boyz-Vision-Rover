import os

files_to_patch = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files_to_patch:
    if not os.path.exists(filepath):
        continue
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # 1. Update pre_x / pre_y
    content = content.replace("float pre_x = cube_x + (dx/len) * 5.0f;", "float pre_x = cube_x + (dx/len) * 10.0f;")
    content = content.replace("float pre_y = cube_y + (dy/len) * 5.0f;", "float pre_y = cube_y + (dy/len) * 10.0f;")
    
    # 2. Update DRIVE_PRE stop distance
    content = content.replace("if (dist_to_cube <= 5.5f) {", "if (dist_to_cube <= 10.5f) {")
    
    # 3. Update VERIFY_TELEMETRY_CUBE trigger distance
    content = content.replace("if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < 8.0f) {", "if (sqrt(pow(c_x - x, 2) + pow(c_y - y, 2)) < 13.0f) {")
    
    with open(filepath, 'w') as f:
        f.write(content)

print("Patch applied to both rovers.")
