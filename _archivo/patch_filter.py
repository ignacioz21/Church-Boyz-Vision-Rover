import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_filter = """                        # Filtrar segun modo
                        if "10" in self.mode: b11 = -1
                        if "11" in self.mode: b10 = -1"""
                        
    new_filter = """                        # Filtrar segun modo aislando las asignaciones
                        if "10" in self.mode: 
                            b11 = -1
                            self.assignments[11] = -1
                        if "11" in self.mode: 
                            b10 = -1
                            self.assignments[10] = -1"""
                            
    content = content.replace(old_filter, new_filter)
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
