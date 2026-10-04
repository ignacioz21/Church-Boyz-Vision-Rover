import os

filepath = "/home/mamalona/.gemini/antigravity/brain/265eda50-6118-4869-a448-f568e998cd32/walkthrough.md"
with open(filepath, 'r') as f:
    content = f.read()

new_section = """
## Iteración 5: Navegación Híbrida Verdadera y Prevención de Colisiones (02 de Octubre)
*   **Problema:** Los rovers chocaban físicamente (las pinzas se solapaban) porque el sistema de evasión APF del ESP32 tenía un margen muy pequeño. Además, si un cubo estaba cerca de la pared, el rover cedía el control muy rápido al ESP32, girando hacia el lado contrario, o se salía del mapa persiguiendo detecciones fantasma (reflejos fuera del grid).
*   **Solución (Cerebro Digital Python):**
    *   **Evasión Dinámica en UI:** `central_brain.py` ahora calcula matemáticamente si la línea de trayectoria de R10 se interseca con R11. Si lo hace, inyecta un punto de evasión lateral (`p_avoid`) que curva la ruta.
    *   **True Hybrid Navigation:** El Cerebro de Python conduce los rovers por WiFi (`M,velL,velR`) a lo largo de toda la pista evadiendo obstáculos, y solo cuando el rover pisa exactamente el "Punto de Pre-Aproximación" (idealmente de espaldas a la pared y frente al cubo), le transfiere el mando al ESP32 (`Fase Final ESP32: 10:3`) para que empuje con precisión milimétrica, eliminando los giros erráticos.
    *   **Filtro Anti-Fantasmas:** Se agregó una barrera de código (`[0 <= col <= grid_cols]`) para ignorar reflejos o falsos cubos detectados fuera de los bordes físicos del tablero.
*   **Validación:**
    *   Verificado mediante logs (`TX M` seguido por `TX Fase Final ESP32`) que la entrega de control ocurre solo a `< 3.0` unidades del punto de pre-aproximación.
"""

if "Iteración 5: Navegación Híbrida Verdadera" not in content:
    content = content + "\n" + new_section
    with open(filepath, 'w') as f:
        f.write(content)
        
print("Walkthrough updated")
