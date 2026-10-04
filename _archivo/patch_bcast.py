import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Change ROVER_BCAST to standard local broadcast
    content = content.replace('ROVER_BCAST = "192.168.88.255"', 'ROVER_BCAST = "255.255.255.255"')

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Broadcast IP patched")
