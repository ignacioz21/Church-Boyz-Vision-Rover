import os

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # The broken block currently looks like:
    #             const char* ansi = "";
    #             const char* reset = "";
    #             if (simulate_routes_mode && is_path_10) ansi = ""; // Amarillo
    #             else if (simulate_routes_mode && is_path_11) ansi = ""; // Morado
    #             else if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = "";
    #             else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "";
    #             else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "";
    
    broken_block = """            const char* ansi = "";
            const char* reset = "";
            if (simulate_routes_mode && is_path_10) ansi = ""; // Amarillo
            else if (simulate_routes_mode && is_path_11) ansi = ""; // Morado
            else if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = "";
            else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "";
            else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "";"""
            
    fixed_block = """            const char* ansi = "";
            const char* reset = "\\033[0m";
            if (simulate_routes_mode && is_path_10) ansi = "\\033[33m"; // Amarillo
            else if (simulate_routes_mode && is_path_11) ansi = "\\033[35m"; // Morado
            else if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = "\\033[31m";
            else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "\\033[32m";
            else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "\\033[34m";"""

    if broken_block in content:
        content = content.replace(broken_block, fixed_block)
        with open(filepath, 'w') as f:
            f.write(content)
        print(f"Fixed {filepath}")
    else:
        print(f"Broken block not found in {filepath}")

fix_file("rover_qa/rover_1/src/navigation.cpp")
fix_file("rover_qa/rover_2/src/navigation.cpp")
