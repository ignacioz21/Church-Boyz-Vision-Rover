import json

with open("vision-system/vision/config_vision.json", "r") as f:
    config = json.load(f)

# 1. Bajar el umbral de saturación (croma) para que vea el azul aunque esté opaco
if "deteccion_cubos" in config:
    config["deteccion_cubos"]["croma_minimo"] = 8.0

# 2. Arreglar la cámara (iluminación y colores)
if "camara" in config and "ajustes" in config["camara"]:
    # Subir exposición para que no se vea tan oscuro (500 -> 600)
    config["camara"]["ajustes"]["exposicion"]["valor"] = 600
    
    # Liberar el balance de blancos a AUTOMATICO. 
    # Esto elimina el tinte amarillo de los focos de casa que se está "comiendo" el color azul.
    config["camara"]["ajustes"]["balance_blancos"]["fijar"] = False

with open("vision-system/vision/config_vision.json", "w") as f:
    json.dump(config, f, indent=2)

