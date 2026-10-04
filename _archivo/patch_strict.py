import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # In compute_assignment
    old_assign_mode = """                if "11" in getattr(self, "mode", ""): cost_matrix[0][i] = 99999.0
                if "10" in getattr(self, "mode", ""): cost_matrix[1][i] = 99999.0"""
                
    new_assign_mode = """                if getattr(self, "mode", "") == "EXEC_11": cost_matrix[0][i] = 99999.0
                if getattr(self, "mode", "") == "EXEC_10": cost_matrix[1][i] = 99999.0"""
                
    content = content.replace(old_assign_mode, new_assign_mode)
    
    # In run
    old_filter = """                        if "10" in self.mode: 
                            b11 = -1
                            self.assignments[11] = -1
                        if "11" in self.mode: 
                            b10 = -1
                            self.assignments[10] = -1"""
                            
    new_filter = """                        if self.mode == "EXEC_10": 
                            b11 = -1
                            self.assignments[11] = -1
                        if self.mode == "EXEC_11": 
                            b10 = -1
                            self.assignments[10] = -1"""
                            
    content = content.replace(old_filter, new_filter)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
