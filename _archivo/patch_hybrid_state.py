import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old = """            if target_idx == -1:
                # No hay cubos asignados (termino o no hay)
                continue"""
                
    new = """            if target_idx == -1:
                # No hay cubos asignados (termino o no hay)
                self.states[rid] = "STANDBY"
                continue"""
                
    content = content.replace(old, new)
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
