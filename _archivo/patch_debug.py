import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Add debug print after compute_assignment
    old_call = """        # Calcular matriz de costos
        b10, b11, rx10, ry10, rx11, ry11 = self.compute_assignment(msg)"""
        
    new_call = """        # Calcular matriz de costos
        b10, b11, rx10, ry10, rx11, ry11 = self.compute_assignment(msg)
        print(f"DEBUG: Asignaciones en crudo b10={b10}, b11={b11}, modo={self.mode}")"""
        
    content = content.replace(old_call, new_call)

    # Add debug print before process_hybrid
    old_process = """        self.process_hybrid(msg, routes)"""
    new_process = """        print(f"DEBUG: Asignaciones post-filtro 10:{self.assignments[10]}, 11:{self.assignments[11]}")
        self.process_hybrid(msg, routes)"""
        
    content = content.replace(old_process, new_process)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Debug patched")
