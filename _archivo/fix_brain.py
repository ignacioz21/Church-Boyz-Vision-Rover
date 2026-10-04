with open("central_brain.py", "r") as f:
    lines = f.readlines()

new_lines = []
for i, line in enumerate(lines):
    if "try: web_log(\"TX\", cmd.decode('utf-8'))" in line:
        continue
    if "except: pass" in line:
        continue
    if line.strip() == "self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))":
        indent = line[:len(line) - len(line.lstrip())]
        new_lines.append(line)
        new_lines.append(indent + "try: web_log('TX', cmd.decode('utf-8'))\n")
        new_lines.append(indent + "except: pass\n")
    elif "self.udp_sock.sendto(f\"{rid}:M,{ml:.2f},{mr:.2f}\".encode(), (ROVER_BCAST, ROVER_PORT))" in line:
        indent = line[:len(line) - len(line.lstrip())]
        new_lines.append(indent + f"cmd = f'{{rid}}:M,{{ml:.2f}},{{mr:.2f}}'.encode()\n")
        new_lines.append(indent + "self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))\n")
    else:
        new_lines.append(line)

with open("central_brain.py", "w") as f:
    f.writelines(new_lines)

