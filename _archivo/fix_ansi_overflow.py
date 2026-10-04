import os

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # We need to replace the loop printing logic.
    # The old block:
    old_block = """            if (ansi[0] != '\\0') {
                udpMap.printf("%s%c%s", ansi, c, reset);
            } else {
                udpMap.print(c);
            }

            if (in_margin && !is_path && c == '#') udpMap.print('#'); 
            else udpMap.print(' ');"""

    new_block = """            if (ansi[0] != '\\0') {
                udpMap.printf("%s%c", ansi, c);
            } else {
                udpMap.printf("\\033[0m%c", c);
            }

            if (in_margin && !is_path && c == '#') udpMap.print('#'); 
            else udpMap.print(' ');"""
            
    # Wait, if we use `\033[0m` for every normal char, it still adds 4 bytes per normal char! That will also overflow!
    # Let's keep a state variable!
    pass

