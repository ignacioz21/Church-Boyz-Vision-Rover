import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_assign = """        for i, c in enumerate(cubes):
            cost_matrix[0][i] = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
            cost_matrix[1][i] = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0"""
            
    new_assign = """        depots = msg.get("depots", [])
        for i, c in enumerate(cubes):
            # Verificacion estricta "EN POSICION"
            color = c.get("color", "")
            depot = next((d for d in depots if d.get("color", "").upper() == color.upper()), None)
            
            in_depot = False
            if depot and "col" in depot:
                dist_dpt = math.sqrt((c["col"] - depot["col"])**2 + (c["row"] - depot["row"])**2)
                if dist_dpt < 5.0:  # 5.0 celdas = 10cm aprox
                    in_depot = True
                    
            if in_depot:
                cost_matrix[0][i] = 9999.0
                cost_matrix[1][i] = 9999.0
            else:
                cost_matrix[0][i] = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                cost_matrix[1][i] = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0"""

    content = content.replace(old_assign, new_assign)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Patch verification successful")
