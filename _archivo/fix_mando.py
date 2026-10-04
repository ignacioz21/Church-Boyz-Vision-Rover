with open('mando.py', 'r') as f:
    text = f.read()

find_str = """        if cmd.strip():
            c = cmd.strip()[0].encode()"""
            
replace_str = """        if cmd.strip():
            if cmd.lower().startswith('d'):
                try:
                    if ':' in cmd:
                        val = float(cmd.split(':')[1])
                    else:
                        val = float(input("Distancia en bloques (ej 8.5): "))
                    payload = f"D:{val}".encode()
                    for d in destinos:
                        try: sock.sendto(payload, (d, 8889))
                        except Exception: pass
                    print(f"-> Distancia {val} enviada.")
                    continue
                except:
                    print("Error formato.")
                    continue
                    
            c = cmd.strip()[0].encode()"""

text = text.replace(find_str, replace_str)

with open('mando.py', 'w') as f:
    f.write(text)

