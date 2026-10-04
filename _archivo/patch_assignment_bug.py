import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Inject mode filter inside compute_assignment
    old_assign = """                cost_matrix[0][i] = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                cost_matrix[1][i] = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0"""
                
    new_assign = """                cost_matrix[0][i] = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                cost_matrix[1][i] = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0
                
                # Deshabilitar costos si el usuario pidio ejecutar solo un rover
                if "11" in getattr(self, "mode", ""): cost_matrix[0][i] = 99999.0
                if "10" in getattr(self, "mode", ""): cost_matrix[1][i] = 99999.0"""
                
    content = content.replace(old_assign, new_assign)
    
    # 2. Fix the out-of-bounds target_idx bug in process_hybrid
    old_hybrid = """        for rid in [10, 11]:
            target_idx = self.assignments[rid]
            
            if target_idx == -1:"""
            
    new_hybrid = """        for rid in [10, 11]:
            target_idx = self.assignments[rid]
            if target_idx >= len(cubes): target_idx = -1
            
            if target_idx == -1:"""
            
    content = content.replace(old_hybrid, new_hybrid)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Patch applied")
