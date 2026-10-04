import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Disable silent exceptions
    content = content.replace("except Exception as e:\n                pass", "except Exception as e:\n                import traceback\n                traceback.print_exc()\n                time.sleep(1)")

    # Replace ["pos"]["x"] and ["pos"]["y"] with ["col"] and ["row"]
    content = content.replace('["pos"]["x"]', '["col"]')
    content = content.replace('["pos"]["y"]', '["row"]')

    # Fix heading to theta
    content = content.replace('["heading"]', '["theta"]')

    # Fix margin 
    content = content.replace('margin = round(grid["margin"] / scale)', 'margin = round(4.0 / scale)')

    # Fix cube and depot colors (RED vs red)
    content = content.replace('color == "RED"', 'color.upper() == "RED"')
    content = content.replace('color == "GREEN"', 'color.upper() == "GREEN"')
    content = content.replace('color == "BLUE"', 'color.upper() == "BLUE"')

    # Fix .get("pos") checks
    content = content.replace('if d.get("pos") and', 'if "col" in d and')
    content = content.replace('if cb.get("pos") and', 'if "col" in cb and')
    content = content.replace('if r.get("pos") and', 'if "col" in r and')

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
