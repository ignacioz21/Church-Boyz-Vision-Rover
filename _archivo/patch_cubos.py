import sys

with open('vision-system/vision/detectors/cubos.py', 'r') as f:
    content = f.read()

find_str = """        region = (etiquetas == etiqueta).astype(np.uint8)
        matiz, croma = matiz_y_croma(cv2.mean(lab, mask=region)[:3])
        color = clasificar(matiz, cfg)"""

replace_str = """        region = (etiquetas == etiqueta).astype(np.uint8)
        matiz, croma = matiz_y_croma(cv2.mean(lab, mask=region)[:3])
        color = clasificar(matiz, cfg)
        print(f"[DEBUG] Cubo mancha detectada: area={area} (ref={area_cara:.1f}, min={area_cara * dc.area_minima_relativa:.1f}, max={area_cara * dc.area_maxima_relativa:.1f}) croma={croma:.1f} matiz={matiz:.1f} color={color}")"""

if find_str in content:
    content = content.replace(find_str, replace_str)
    with open('vision-system/vision/detectors/cubos.py', 'w') as f:
        f.write(content)
    print("Patched successfully")
else:
    print("String not found")

