import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_calc = """        def calc_route(rx, ry, cube_idx):
            if cube_idx == -1 or rx > 900.0: return None
            if cube_idx >= len(cubes): return None
            c = cubes[cube_idx]
            color = c["color"]
            d = None
            for dpt in depots:
                if dpt.get("color", "").upper() == color.upper():
                    d = dpt
                    break
            if not d: return None
            
            dx = c["col"] - d["col"]
            dy = c["row"] - d["row"]
            length = math.sqrt(dx**2 + dy**2) or 1.0
            
            pre_x = c["col"] + (dx/length) * PRE_APPROACH_DIST
            pre_y = c["row"] + (dy/length) * PRE_APPROACH_DIST
            pre_x = max(3.0, min(grid_cols - 3.0, pre_x))
            pre_y = max(3.0, min(grid_rows - 3.0, pre_y))
            
            # El objetivo de push debe estar MAS ALLA del centro del deposito (signo negativo)
            push_x = d["col"] - (dx/length) * 3.0
            push_y = d["row"] - (dy/length) * 3.0
            
            return {
                "p1": (rx, ry),
                "p2": (pre_x, pre_y),
                "p3": (d["col"], d["row"]),
                "cube": (c["col"], c["row"]),
                "push": (push_x, push_y)
            }

        routes[10] = calc_route(rx_10, ry_10, best_10)
        routes[11] = calc_route(rx_11, ry_11, best_11)"""

    new_calc = """        def calc_route(rid, rx, ry, cube_idx):
            if cube_idx == -1 or rx > 900.0: return None
            if cube_idx >= len(cubes): return None
            c = cubes[cube_idx]
            color = c["color"]
            d = None
            for dpt in depots:
                if dpt.get("color", "").upper() == color.upper():
                    d = dpt
                    break
            if not d: return None
            
            dx = c["col"] - d["col"]
            dy = c["row"] - d["row"]
            length = math.sqrt(dx**2 + dy**2) or 1.0
            
            pre_x = c["col"] + (dx/length) * PRE_APPROACH_DIST
            pre_y = c["row"] + (dy/length) * PRE_APPROACH_DIST
            pre_x = max(3.0, min(grid_cols - 3.0, pre_x))
            pre_y = max(3.0, min(grid_rows - 3.0, pre_y))
            
            # El objetivo de push debe estar MAS ALLA del centro del deposito
            push_x = d["col"] - (dx/length) * 3.0
            push_y = d["row"] - (dy/length) * 3.0
            
            p_avoid = None
            # Check collision with OTHER rover
            other_rx = rx_11 if rid == 10 else rx_10
            other_ry = ry_11 if rid == 10 else ry_10
            
            if other_rx < 900.0:
                # Vector de p1 a p2
                vx = pre_x - rx
                vy = pre_y - ry
                v_len = math.sqrt(vx**2 + vy**2)
                if v_len > 0:
                    vx /= v_len
                    vy /= v_len
                    # Proyeccion del obstaculo en la recta
                    wx = other_rx - rx
                    wy = other_ry - ry
                    proj = wx*vx + wy*vy
                    
                    # Si el obstaculo esta ENTRE el rover y su destino
                    if 0 < proj < v_len:
                        closest_x = rx + proj * vx
                        closest_y = ry + proj * vy
                        dist_to_line = math.sqrt((other_rx - closest_x)**2 + (other_ry - closest_y)**2)
                        
                        # Si esta demasiado cerca de la linea de trayectoria (e.g. 10.0 unidades, considerando las pinzas de 4 bloques)
                        if dist_to_line < 10.0:
                            # Calcular un punto de evasion lateral
                            # Vector perpendicular
                            nx = -vy
                            ny = vx
                            
                            # Decidir a que lado desviar (el que requiera menor desvio)
                            if (wx*nx + wy*ny) > 0:
                                # Obstaculo esta "a la derecha" (direccion normal), evadir hacia la izquierda (-normal)
                                nx = -nx
                                ny = -ny
                                
                            p_avoid_x = closest_x + nx * 10.0
                            p_avoid_y = closest_y + ny * 10.0
                            
                            # Restringir a los bordes de la cancha
                            p_avoid_x = max(3.0, min(grid_cols - 3.0, p_avoid_x))
                            p_avoid_y = max(3.0, min(grid_rows - 3.0, p_avoid_y))
                            
                            p_avoid = (p_avoid_x, p_avoid_y)
                            
            return {
                "p1": (rx, ry),
                "p_avoid": p_avoid,
                "p2": (pre_x, pre_y),
                "p3": (d["col"], d["row"]),
                "cube": (c["col"], c["row"]),
                "push": (push_x, push_y)
            }

        routes[10] = calc_route(10, rx_10, ry_10, best_10)
        routes[11] = calc_route(11, rx_11, ry_11, best_11)"""
        
    content = content.replace(old_calc, new_calc)
    
    # Update map export to include p_avoid
    old_export = """            for rid in [10, 11]:
                if routes.get(rid):
                    r = routes[rid]
                    rt[str(rid)] = {"p1": [r["p1"][0], r["p1"][1]], "p2": [r["p2"][0], r["p2"][1]], "p3": [r["p3"][0], r["p3"][1]]}"""
                    
    new_export = """            for rid in [10, 11]:
                if routes.get(rid):
                    r = routes[rid]
                    rt[str(rid)] = {
                        "p1": [r["p1"][0], r["p1"][1]], 
                        "p_avoid": [r["p_avoid"][0], r["p_avoid"][1]] if r.get("p_avoid") else None,
                        "p2": [r["p2"][0], r["p2"][1]], 
                        "p3": [r["p3"][0], r["p3"][1]]
                    }"""
                    
    content = content.replace(old_export, new_export)
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Avoidance logic patched in Brain")
