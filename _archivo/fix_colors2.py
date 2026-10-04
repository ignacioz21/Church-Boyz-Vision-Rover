import os

def fix(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    for i in range(len(lines)):
        if 'const char* reset = "";' in lines[i]:
            lines[i] = '            const char* reset = "\\033[0m";\n'
        elif 'if (simulate_routes_mode && is_path_10) ansi = ""; // Amarillo' in lines[i]:
            lines[i] = '            if (simulate_routes_mode && is_path_10) ansi = "\\033[33m"; // Amarillo\n'
        elif 'else if (simulate_routes_mode && is_path_11) ansi = ""; // Morado' in lines[i]:
            lines[i] = '            else if (simulate_routes_mode && is_path_11) ansi = "\\033[35m"; // Morado\n'
        elif "else if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = " in lines[i] and 'ansi = "";' in lines[i]:
            lines[i] = '            else if (c == \'R\' || c == \'r\' || (is_path && active_hunt_color == COLOR_RED)) ansi = "\\033[31m";\n'
        elif "else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = " in lines[i] and 'ansi = "";' in lines[i]:
            lines[i] = '            else if (c == \'G\' || c == \'g\' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "\\033[32m";\n'
        elif "else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = " in lines[i] and 'ansi = "";' in lines[i]:
            lines[i] = '            else if (c == \'B\' || c == \'b\' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "\\033[34m";\n'

    with open(filepath, 'w') as f:
        f.writelines(lines)

fix("rover_qa/rover_1/src/navigation.cpp")
fix("rover_qa/rover_2/src/navigation.cpp")
