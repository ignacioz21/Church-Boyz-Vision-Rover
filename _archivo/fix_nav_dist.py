import os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    # If it's already there, remove it to be safe
    if "static float current_pre_dist" in content:
        # Already exists, wait, it shouldn't exist because the compilation failed!
        pass
    
    find_str = """#include <WiFiUdp.h>"""
    
    replace_str = """#include <WiFiUdp.h>

static float current_pre_dist = 10.0f;
void setPreApproachDistance(float blocks) { current_pre_dist = blocks; }
float getPreApproachDistance() { return current_pre_dist; }"""
    
    content = content.replace(find_str, replace_str)
    
    with open(filepath, 'w') as f: f.write(content)

print("Fixed missing declarations in navigation.cpp")
