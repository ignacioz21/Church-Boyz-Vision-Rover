import os
import re

def strip_map(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Find printMap function and delete it
    print_map_pattern = r'void printMap\(const TelemetrySnapshot &snap\) \{.*?\n\}\n\n'
    content = re.sub(print_map_pattern, '', content, flags=re.DOTALL)
    
    # Delete simulateRoutesTask
    sim_routes_pattern = r'void simulateRoutesTask\(\) \{.*?\}\n'
    content = re.sub(sim_routes_pattern, '', content, flags=re.DOTALL)
    
    # Also clean up the globals at the top
    globals_pattern = r'bool simulate_routes_mode = false;.*?float sim_r11_p3_y = -1;\n'
    content = re.sub(globals_pattern, '', content, flags=re.DOTALL)
    
    with open(filepath, 'w') as f:
        f.write(content)

strip_map("rover_qa/rover_1/src/navigation.cpp")
strip_map("rover_qa/rover_2/src/navigation.cpp")
