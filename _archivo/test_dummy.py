class DummyBrain:
    def __init__(self):
        self.mode = "EXEC_11"
        self.assignments = {10: -1, 11: -1}

    def compute_assignment(self):
        cost_matrix = [[9999.0]*3 for _ in range(2)]
        rx_10, ry_10 = 10, 10
        rx_11, ry_11 = 20, 20
        c = {"col": 15, "row": 15}
        
        cost_matrix[0][0] = 7.07
        cost_matrix[1][0] = 7.07
        
        if "11" in self.mode: cost_matrix[0][0] = 99999.0
        if "10" in self.mode: cost_matrix[1][0] = 99999.0
        
        min_total = 99999.0
        best_10, best_11 = -1, -1
        
        for c10 in range(3):
            for c11 in range(3):
                if c10 == c11: continue
                cost0 = cost_matrix[0][c10]
                cost1 = cost_matrix[1][c11]
                if self.assignments[10] == c10: cost0 *= 0.1
                if self.assignments[11] == c11: cost1 *= 0.1
                
                total = cost0 + cost1
                if total < min_total:
                    min_total = total
                    best_10, best_11 = c10, c11
                    
        return best_10, best_11

d = DummyBrain()
b10, b11 = d.compute_assignment()
print(f"b10={b10}, b11={b11}")
