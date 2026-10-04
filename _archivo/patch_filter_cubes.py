import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Add filter logic right after msg.get("cubes", [])
    old_hybrid = """    def process_hybrid(self, msg):
        # Solucion Hibrida: El Cerebro evalua el estado global y asigna tareas de alto nivel.
        # Los ESP32 ejecutan la maniobra local (hunt_cube) usando sus sensores fisicos para precision cero-latencia.
        cubes = msg.get("cubes", [])"""
        
    new_hybrid = """    def process_hybrid(self, msg):
        # Solucion Hibrida: El Cerebro evalua el estado global y asigna tareas de alto nivel.
        # Los ESP32 ejecutan la maniobra local (hunt_cube) usando sus sensores fisicos para precision cero-latencia.
        raw_cubes = msg.get("cubes", [])
        grid_cols = msg.get("grid", {}).get("cols", 40)
        grid_rows = msg.get("grid", {}).get("rows", 25)
        # Ignorar fantasmas fuera de la cancha
        cubes = [c for c in raw_cubes if 0 <= c.get("col", -1) <= grid_cols and 0 <= c.get("row", -1) <= grid_rows]"""
        
    content = content.replace(old_hybrid, new_hybrid)
    
    # Also filter in compute_assignment
    old_assign = """    def compute_assignment(self, msg):
        cubes = msg.get("cubes", [])
        active_cubes = [(i, c) for i, c in enumerate(cubes) if c.get("color", "").upper() != "YELLOW"]"""
        
    new_assign = """    def compute_assignment(self, msg):
        raw_cubes = msg.get("cubes", [])
        grid_cols = msg.get("grid", {}).get("cols", 40)
        grid_rows = msg.get("grid", {}).get("rows", 25)
        cubes = [c for c in raw_cubes if 0 <= c.get("col", -1) <= grid_cols and 0 <= c.get("row", -1) <= grid_rows]
        active_cubes = [(i, c) for i, c in enumerate(cubes) if c.get("color", "").upper() != "YELLOW"]"""
        
    content = content.replace(old_assign, new_assign)
    
    # Also filter in compute_routes
    old_route = """    def compute_routes(self, msg, best_10, best_11, rx_10, ry_10, rx_11, ry_11):
        cubes = msg.get("cubes", [])
        depots = msg.get("depots", [])"""
        
    new_route = """    def compute_routes(self, msg, best_10, best_11, rx_10, ry_10, rx_11, ry_11):
        raw_cubes = msg.get("cubes", [])
        grid_cols = msg.get("grid", {}).get("cols", 40)
        grid_rows = msg.get("grid", {}).get("rows", 25)
        cubes = [c for c in raw_cubes if 0 <= c.get("col", -1) <= grid_cols and 0 <= c.get("row", -1) <= grid_rows]
        depots = msg.get("depots", [])"""
        
    content = content.replace(old_route, new_route)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Filtered out-of-bounds phantom cubes")
