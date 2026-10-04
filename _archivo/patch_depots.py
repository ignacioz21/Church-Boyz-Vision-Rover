import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # In compute_routes:
    # d = depots[depot_idx] -> search for depot matching color
    old_depot_search = """            depot_idx = 0 if color.upper() == "RED" else (1 if color.upper() == "GREEN" else 2)
            if depot_idx >= len(depots): return None
            d = depots[depot_idx]"""
            
    new_depot_search = """            d = None
            for dpt in depots:
                if dpt.get("color", "").upper() == color.upper():
                    d = dpt
                    break
            if not d: return None"""
            
    content = content.replace(old_depot_search, new_depot_search)
    
    # In print_map:
    # c = depots[i] -> c = 'r', 'g', 'b' depending on color
    old_print_depot = """                for i, d in enumerate(msg.get("depots", [])):
                    if "col" in d and round(d["col"]/scale) == x and round(d["row"]/scale) == y:
                        c = depots[i]
                for i, cb in enumerate(msg.get("cubes", [])):
                    if "col" in cb and round(cb["col"]/scale) == x and round(cb["row"]/scale) == y:
                        c = cubes_colors[i]"""
                        
    new_print_depot = """                for d in msg.get("depots", []):
                    if "col" in d and round(d["col"]/scale) == x and round(d["row"]/scale) == y:
                        c = d["color"][0].lower() if d.get("color") else 'd'
                for cb in msg.get("cubes", []):
                    if "col" in cb and round(cb["col"]/scale) == x and round(cb["row"]/scale) == y:
                        c = cb["color"][0].upper() if cb.get("color") else 'C'"""
                        
    content = content.replace(old_print_depot, new_print_depot)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
