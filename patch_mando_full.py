with open('mando.py', 'r') as f: text = f.read()

find_str = """            c = cmd.strip()[0].encode()
            for d in destinos:"""
            
replace_str = """            # Si el comando contiene dos puntos (ej. 10:6), mandamos todo el string. Si no, solo el primer caracter
            c = cmd.strip().encode() if ':' in cmd else cmd.strip()[0].encode()
            for d in destinos:"""
            
text = text.replace(find_str, replace_str)
with open('mando.py', 'w') as f: f.write(text)

print("mando.py patched for targeted strings.")
