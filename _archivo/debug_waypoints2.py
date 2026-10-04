with open("central_brain.py", "r") as f:
    content = f.read()

content = content.replace('target_idx = self.assignments[rid]',
'''target_idx = self.assignments[rid]
            print(f"[DEBUG] Rover {rid} target_idx={target_idx}, route={bool(route)}")''')

with open("central_brain.py", "w") as f:
    f.write(content)
