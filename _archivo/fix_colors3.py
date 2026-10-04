import sys

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()
    
    text = text.replace('const char* reset = "";', 'const char* reset = "\\033[0m";')
    text = text.replace('if (simulate_routes_mode && is_path_10) ansi = ""; // Amarillo', 'if (simulate_routes_mode && is_path_10) ansi = "\\033[33m"; // Amarillo')
    text = text.replace('else if (simulate_routes_mode && is_path_11) ansi = ""; // Morado', 'else if (simulate_routes_mode && is_path_11) ansi = "\\033[35m"; // Morado')
    text = text.replace('else if (c == \'R\' || c == \'r\' || (is_path && active_hunt_color == COLOR_RED)) ansi = "";', 'else if (c == \'R\' || c == \'r\' || (is_path && active_hunt_color == COLOR_RED)) ansi = "\\033[31m";')
    text = text.replace('else if (c == \'G\' || c == \'g\' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "";', 'else if (c == \'G\' || c == \'g\' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "\\033[32m";')
    text = text.replace('else if (c == \'B\' || c == \'b\' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "";', 'else if (c == \'B\' || c == \'b\' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "\\033[34m";')

    with open(filepath, 'w') as f:
        f.write(text)

fix_file("rover_qa/rover_1/src/navigation.cpp")
fix_file("rover_qa/rover_2/src/navigation.cpp")
