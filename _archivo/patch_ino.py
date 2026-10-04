import re

def patch_h(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    if "void go_home();" not in content:
        content = content.replace("bool hunt_cube(CubeColor target_color);", "void go_home();\nbool hunt_cube(CubeColor target_color);")
    with open(filepath, 'w') as f:
        f.write(content)

def patch_ino(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
        
    cmd_h = """        } else if (cmd == 'h' || cmd == 'H') {
            Serial.println("\\n[MENU] >>> REGRESANDO A CASA...");
            go_home();"""
            
    if "cmd == 'h'" not in content:
        content = content.replace("        } else if (cmd == '3') {", cmd_h + "\n        } else if (cmd == '3') {")
        
    with open(filepath, 'w') as f:
        f.write(content)

patch_h("rover_qa/rover_1/include/navigation.h")
patch_h("rover_qa/rover_2/include/navigation.h")
patch_ino("rover_qa/rover_1/rover_1.ino")
patch_ino("rover_qa/rover_2/rover_2.ino")
