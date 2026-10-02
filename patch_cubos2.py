import sys

with open('vision-system/vision/detectors/cubos.py', 'r') as f:
    content = f.read()

find_str = """        area = int(stats[etiqueta, cv2.CC_STAT_AREA])
        if not (area_cara * dc.area_minima_relativa <= area <= area_cara * dc.area_maxima_relativa):
            continue"""

replace_str = """        area = int(stats[etiqueta, cv2.CC_STAT_AREA])
        region_tmp = (etiquetas == etiqueta).astype(np.uint8)
        matiz_t, croma_t = matiz_y_croma(cv2.mean(lab, mask=region_tmp)[:3])
        color_t = clasificar(matiz_t, cfg)
        if area > 100:  # Solo imprimir cosas no muy pequeñas
            print(f"[DEBUG-ALL] Mancha croma={croma_t:.1f} matiz={matiz_t:.1f} color={color_t} area={area} (ref={area_cara:.1f}, min={area_cara * dc.area_minima_relativa:.1f})")
        if not (area_cara * dc.area_minima_relativa <= area <= area_cara * dc.area_maxima_relativa):
            continue"""

if find_str in content:
    content = content.replace(find_str, replace_str)
    with open('vision-system/vision/detectors/cubos.py', 'w') as f:
        f.write(content)
    print("Patched successfully")
else:
    print("String not found")

