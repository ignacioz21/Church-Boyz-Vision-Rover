with open("central_brain.py", "r") as f:
    content = f.read()

content = content.replace('def process_waypoints(self, msg, routes):', 
'''def process_waypoints(self, msg, routes):
        print(f"[DEBUG] process_waypoints called! Mode: {self.mode}")''')

content = content.replace('if "EXEC" in self.mode or self.mode == "AUTO":',
'''print(f"[DEBUG] check EXEC in run. mode={self.mode}")
                        if "EXEC" in self.mode or self.mode == "AUTO":''')

with open("central_brain.py", "w") as f:
    f.write(content)
