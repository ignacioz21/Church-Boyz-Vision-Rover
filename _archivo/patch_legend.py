import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_header = """        out = "\033[H\033[2J" # Clear screen
        out += "=== CEREBRO DIGITAL (CONTROL CENTRALIZADO) ===\\n"
        out += "+" + "--"*map_w + "+\\n\""""
        
    # We will search with a simple string replace for the title
    if 'out += "=== CEREBRO DIGITAL (CONTROL CENTRALIZADO) ===\\n"' in content:
        old_str = 'out += "=== CEREBRO DIGITAL (CONTROL CENTRALIZADO) ===\\n"'
        new_str = 'out += "=== CEREBRO DIGITAL (CONTROL CENTRALIZADO) ===\\n"\n' + \
                  '        out += " LEYENDA:\\n"\n' + \
                  '        out += "  Objetivos: [\\033[31m R/r \\033[0m: Rojo] [\\033[32m G/g \\033[0m: Verde] [\\033[34m B/b \\033[0m: Azul] (Mayus=Cubo, Minus=Deposito)\\n"\n' + \
                  '        out += "  Navegacion: [\\033[33m > \\033[0m: Rover 10] [\\033[35m > \\033[0m: Rover 11]  [*]: Ruta  [+]: Punto de Aproximacion\\n"\n'
        content = content.replace(old_str, new_str)
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Legend added to terminal map")
