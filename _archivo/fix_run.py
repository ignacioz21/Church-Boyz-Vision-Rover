with open("central_brain.py", "r") as f:
    content = f.read()

import re

# Remove the IDLE continue block
content = re.sub(r'if self\.mode == "IDLE":\s*self\.print_map\(msg, \{10: None, 11: None\}\)\s*continue', 
                 '# IDLE block removed to allow process_waypoints to stop rovers', content)

# Remove the "if EXEC in self.mode" condition around process_waypoints
content = re.sub(r'if "EXEC" in self\.mode or self\.mode == "AUTO":\s*self\.process_waypoints\(msg, routes\)',
                 'self.process_waypoints(msg, routes)', content)

with open("central_brain.py", "w") as f:
    f.write(content)

