import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Find where the Y loop starts to insert current_ansi
    content = content.replace("    // Filas\n    for (int y = 0; y < map_h; y++) {", "    // Filas\n    const char* current_ansi = \"\";\n    for (int y = 0; y < map_h; y++) {")

    old_colorear = """            // Colorear e imprimir
            const char* ansi = "";
            const char* reset = "\\033[0m";
            if (simulate_routes_mode && is_path_10) ansi = "\\033[33m"; // Amarillo
            else if (simulate_routes_mode && is_path_11) ansi = "\\033[35m"; // Morado
            else if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = "\\033[31m";
            else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "\\033[32m";
            else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "\\033[34m";
            else if (c == 'O') ansi = "\\033[33m"; // Compañero (Amarillo)
            else if (c == '>' || c == '<' || c == '^' || c == 'v') ansi = "\\033[1;37m"; // Blanco Brillante para el rover
            
            if (ansi[0] != '\\0') {
                udpMap.printf("%s%c%s", ansi, c, reset);
            } else {
                udpMap.print(c);
            }

            if (in_margin && !is_path && c == '#') udpMap.print('#'); 
            else udpMap.print(' ');"""

    new_colorear = """            // Colorear e imprimir
            const char* ansi = "\\033[0m";
            if (simulate_routes_mode && is_path_10) ansi = "\\033[33m"; // Amarillo
            else if (simulate_routes_mode && is_path_11) ansi = "\\033[35m"; // Morado
            else if (c == 'R' || c == 'r' || (is_path && active_hunt_color == COLOR_RED)) ansi = "\\033[31m";
            else if (c == 'G' || c == 'g' || (is_path && active_hunt_color == COLOR_GREEN)) ansi = "\\033[32m";
            else if (c == 'B' || c == 'b' || (is_path && active_hunt_color == COLOR_BLUE)) ansi = "\\033[34m";
            else if (c == 'O') ansi = "\\033[33m"; // Compañero (Amarillo)
            else if (c == '>' || c == '<' || c == '^' || c == 'v') ansi = "\\033[1;37m"; // Blanco Brillante para el rover
            
            if (strcmp(ansi, current_ansi) != 0) {
                udpMap.print(ansi);
                current_ansi = ansi;
            }
            
            udpMap.print(c);

            if (in_margin && !is_path && c == '#') udpMap.print('#'); 
            else udpMap.print(' ');"""

    if old_colorear in content:
        content = content.replace(old_colorear, new_colorear)
        with open(filepath, 'w') as f:
            f.write(content)
        print(f"Patched {filepath}")
    else:
        print(f"Could not find block in {filepath}")

fix_file("rover_qa/rover_1/src/navigation.cpp")
fix_file("rover_qa/rover_2/src/navigation.cpp")
