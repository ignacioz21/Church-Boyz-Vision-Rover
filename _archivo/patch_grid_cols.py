import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # compute_assignment
    content = content.replace('grid_cols = msg.get("grid", {}).get("cols", 40)', 'grid_cols = msg.get("grid", {}).get("cols", 43.0)')
    content = content.replace('grid_rows = msg.get("grid", {}).get("rows", 25)', 'grid_rows = msg.get("grid", {}).get("rows", 28.0)')

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Grid cols fixed")
