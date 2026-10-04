with open("vision-system/vision/vista.py", "r") as f:
    content = f.read()

# 1. Improve JPEG quality from 70 to 95 for less artifacts on the web
content = content.replace("_, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])", 
                          "_, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])")

# 2. Add an unsharp mask filter to the frame before assigning it to the web and cv2.imshow
enchance_code = """
        # --- MEJORA DE CALIDAD (Nitidez y Contraste) ---
        # 1. Unsharp mask para nitidez
        gaussian = cv2.GaussianBlur(lienzo, (0, 0), 2.0)
        lienzo = cv2.addWeighted(lienzo, 1.5, gaussian, -0.5, 0)
        # 2. Ajuste sutil de brillo y contraste
        lienzo = cv2.convertScaleAbs(lienzo, alpha=1.1, beta=10)

        cv2.imshow(self._titulo, lienzo)
"""

content = content.replace("        cv2.imshow(self._titulo, lienzo)", enchance_code)

with open("vision-system/vision/vista.py", "w") as f:
    f.write(content)
