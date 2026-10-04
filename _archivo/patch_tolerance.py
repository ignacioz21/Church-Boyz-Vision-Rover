import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Change tolerance from 5.0 (10 cm) to 2.0 (4 cm)
    content = content.replace("if dist_dpt < 5.0: in_depot = True", "if dist_dpt < 2.0: in_depot = True")

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Tolerance patched to 2.0 (4 cm)")
