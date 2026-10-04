import os

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Find the end of the X loop
    old_end = """            if (in_margin && !is_path && c == '#') udpMap.print('#'); 
            else udpMap.print(' ');
        }
        udpMap.println("|");"""

    new_end = """            if (in_margin && !is_path && c == '#') udpMap.print('#'); 
            else udpMap.print(' ');
        }
        if (strcmp(current_ansi, "\\033[0m") != 0) {
            udpMap.print("\\033[0m");
            current_ansi = "\\033[0m";
        }
        udpMap.println("|");"""

    if old_end in content:
        content = content.replace(old_end, new_end)
        with open(filepath, 'w') as f:
            f.write(content)

fix_file("rover_qa/rover_1/src/navigation.cpp")
fix_file("rover_qa/rover_2/src/navigation.cpp")
