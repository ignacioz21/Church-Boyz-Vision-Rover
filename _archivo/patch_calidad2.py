with open("vision-system/vision/vista.py", "r") as f:
    content = f.read()

# Reemplazar el ajuste sutil por uno mas fuerte
content = content.replace("lienzo = cv2.convertScaleAbs(lienzo, alpha=1.1, beta=10)", 
                          "lienzo = cv2.convertScaleAbs(lienzo, alpha=1.3, beta=50)")

with open("vision-system/vision/vista.py", "w") as f:
    f.write(content)
