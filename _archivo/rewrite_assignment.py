import re

with open("central_brain.py", "r") as f:
    content = f.read()

# Buscamos donde empieza compute_assignment
start_idx = content.find("    def compute_assignment(self, msg):")

# Buscamos donde termina (la funcion siguiente es compute_routes)
end_idx = content.find("    def compute_routes(self, msg, best_10, best_11, rx_10, ry_10, rx_11, ry_11):")

new_func = """    def compute_assignment(self, msg):
        cubes = msg.get("cubes", [])
        rovers = msg.get("rovers", [])
        
        rx_10, ry_10 = 999.0, 999.0
        rx_11, ry_11 = 999.0, 999.0
        for r in rovers:
            if r["id"] == 10: rx_10, ry_10 = r["col"], r["row"]
            elif r["id"] == 11: rx_11, ry_11 = r["col"], r["row"]
                
        depots = msg.get("depots", [])
        
        # 1. Identificar cubos "activos" (los que todavia no estan en su deposito)
        active_cubes = []
        for i, c in enumerate(cubes):
            color = c.get("color", "")
            depot = next((d for d in depots if d.get("color", "").upper() == color.upper()), None)
            in_depot = False
            if depot and "col" in depot:
                dist_dpt = math.sqrt((c["col"] - depot["col"])**2 + (c["row"] - depot["row"])**2)
                if dist_dpt < 5.0: in_depot = True
            
            if not in_depot:
                active_cubes.append((i, c))

        best_10, best_11 = -1, -1
        mode = getattr(self, "mode", "IDLE")

        # 2. Asignacion MODO AISLADO (e1) - Solo Rover 10
        if mode == "EXEC_10":
            if active_cubes:
                best_dist = 99999.0
                for orig_idx, c in active_cubes:
                    d = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                    if self.assignments[10] == orig_idx: d *= 0.1 # Inercia
                    if d < best_dist:
                        best_dist = d
                        best_10 = orig_idx

        # 3. Asignacion MODO AISLADO (e2) - Solo Rover 11
        elif mode == "EXEC_11":
            if active_cubes:
                best_dist = 99999.0
                for orig_idx, c in active_cubes:
                    d = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0
                    if self.assignments[11] == orig_idx: d *= 0.1
                    if d < best_dist:
                        best_dist = d
                        best_11 = orig_idx

        # 4. Asignacion MODO PAREJA (ee o AUTO)
        else:
            if len(active_cubes) == 1:
                orig_idx, c = active_cubes[0]
                d10 = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                d11 = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0
                if self.assignments[10] == orig_idx: d10 *= 0.1
                if self.assignments[11] == orig_idx: d11 *= 0.1
                if d10 <= d11: best_10 = orig_idx
                else: best_11 = orig_idx
            
            elif len(active_cubes) >= 2:
                min_total = 99999.0
                for idx10, c10 in active_cubes:
                    for idx11, c11 in active_cubes:
                        if idx10 == idx11: continue
                        d10 = math.sqrt((c10["col"] - rx_10)**2 + (c10["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                        d11 = math.sqrt((c11["col"] - rx_11)**2 + (c11["row"] - ry_11)**2) if rx_11 < 900 else 9999.0
                        if self.assignments[10] == idx10: d10 *= 0.1
                        if self.assignments[11] == idx11: d11 *= 0.1
                        
                        if d10 + d11 < min_total:
                            min_total = d10 + d11
                            best_10 = idx10
                            best_11 = idx11

        self.assignments[10] = best_10
        self.assignments[11] = best_11
        return best_10, best_11, rx_10, ry_10, rx_11, ry_11

"""

new_content = content[:start_idx] + new_func + content[end_idx:]

with open("central_brain.py", "w") as f:
    f.write(new_content)

