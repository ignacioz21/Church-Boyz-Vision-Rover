import os

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Find the row loop
    old_loop = "    // Filas\n    const char* current_ansi = \"\";\n    for (int y = 0; y < map_h; y++) {"
    new_loop = "    // Filas\n    const char* current_ansi = \"\";\n    int rows_sent = 0;\n    for (int y = 0; y < map_h; y++) {"

    if old_loop in content:
        content = content.replace(old_loop, new_loop)

    old_row_end = """        }
        if (strcmp(current_ansi, "\\033[0m") != 0) {
            udpMap.print("\\033[0m");
            current_ansi = "\\033[0m";
        }
        udpMap.println("|");"""

    new_row_end = """        }
        if (strcmp(current_ansi, "\\033[0m") != 0) {
            udpMap.print("\\033[0m");
            current_ansi = "\\033[0m";
        }
        udpMap.println("|");
        rows_sent++;
        if (rows_sent >= 5) {
            udpMap.endPacket();
            udpMap.beginPacket(VISION_HOST, 8888);
            rows_sent = 0;
        }"""

    if old_row_end in content:
        content = content.replace(old_row_end, new_row_end)
        with open(filepath, 'w') as f:
            f.write(content)
        print(f"Patched {filepath}")
    else:
        print(f"Could not find block in {filepath}")

fix_file("rover_qa/rover_1/src/navigation.cpp")
fix_file("rover_qa/rover_2/src/navigation.cpp")
